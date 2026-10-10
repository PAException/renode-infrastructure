//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System;
using System.Collections.Generic;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    /// <summary>
    /// S32K1xx FlexTimer Module (FTM), input capture subset, with a SENT (SAE J2716) transmitter per channel.
    ///
    /// Modelled: the counter (SC.CLKS/PS, CNT, CNTIN, MOD) with its overflow flag and interrupt, and
    /// per channel input capture on falling input edges (CnSC.ELSnB with MSn = 00) setting CnV and CHF,
    /// with the channel interrupts of the channel pairs. PWM, output compare, combine/dual edge modes,
    /// faults, quadrature decoding and register synchronization are not; their registers are plain storage.
    ///
    /// The SENT transmitters stand in for sensors on the channel inputs: SetSentFrame() gives a channel the
    /// nibbles to send (status, data, CRC), which it repeats back to back, each frame starting with the
    /// 56 tick sync pulse. Each nibble is the time between two falling edges (12 + value ticks).
    ///
    /// Edge timing: with serial execution, a machine's timers only fire at the sync points between quanta,
    /// while SENT edges are 36..168 us apart. So edges are computed from the frame schedule, and each one
    /// is captured with the counter value of its exact time - but delivered (CHF, interrupt) late, at the
    /// next point the model gets to run. Edges that fell due meanwhile are delivered one after another:
    /// the next one as soon as the firmware clears CHF of the previous. While an edge is pending, CNT reads
    /// return its capture time, like a CNT read in an interrupt handler that runs right at the edge.
    /// Firmware that timestamps edges via CnV or via CNT in its interrupt handler therefore measures exact
    /// intervals, only later than in reality.
    ///
    /// Edges are only generated while the channel's interrupt is enabled (CHIE), as a SENT receiver only
    /// listens on that channel. Enabling it restarts the transmission with a falling edge right away, like
    /// the edge a receiver finds pending when it starts listening to a sensor that was sending all along.
    /// </summary>
    public class S32K1XX_FTM : BasicDoubleWordPeripheral, IKnownSize
    {
        public S32K1XX_FTM(IMachine machine, ulong fixedFrequencyClock = 8000000, ulong inputClock = 48000000) : base(machine)
        {
            this.fixedFrequencyClock = fixedFrequencyClock;
            this.inputClock = inputClock;

            Channel01IRQ = new GPIO();
            Channel23IRQ = new GPIO();
            Channel45IRQ = new GPIO();
            Channel67IRQ = new GPIO();
            OverflowIRQ = new GPIO();

            counter = new LimitTimer(machine.ClockSource, fixedFrequencyClock, this, "counter", limit: 0x10000,
                direction: Direction.Ascending, enabled: false, workMode: WorkMode.Periodic, eventEnabled: true);
            counter.LimitReached += OnCounterOverflow;

            channelFlag = new IFlagRegisterField[NumberOfChannels];
            channelInterruptEnable = new IFlagRegisterField[NumberOfChannels];
            modeSelect = new IValueRegisterField[NumberOfChannels];
            edgeSelect = new IValueRegisterField[NumberOfChannels];
            channelValue = new IValueRegisterField[NumberOfChannels];
            pendingCapture = new uint?[NumberOfChannels];
            transmitters = new SentTransmitter[NumberOfChannels];
            for(var channel = 0; channel < NumberOfChannels; channel++)
            {
                transmitters[channel] = new SentTransmitter(this, machine, channel);
            }

            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            counter.Reset();
            foreach(var transmitter in transmitters)
            {
                transmitter.Stop();
            }
            for(var channel = 0; channel < NumberOfChannels; channel++)
            {
                pendingCapture[channel] = null;
            }
            UpdateCounter();
            UpdateInterrupts();
        }

        /// <summary>
        /// Sets the nibbles a SENT sensor on the channel input sends after each sync pulse: status, data
        /// nibbles, CRC. A frame being sent is completed first. A connected sensor sends as soon as it has a frame.
        /// </summary>
        public void SetSentFrame(int channel, uint[] nibbles, double tickMicroseconds = 3.0)
        {
            if(!IsValidChannel(channel))
            {
                return;
            }
            if(nibbles.Any(nibble => nibble > 0xF))
            {
                this.Log(LogLevel.Warning, "SENT nibbles have to be 0..15, frame for channel {0} ignored", channel);
                return;
            }
            transmitters[channel].SetFrame(nibbles, tickMicroseconds);
        }

        /// <summary>Connects or disconnects the SENT sensor on the channel input. A disconnected sensor sends no edges.</summary>
        public void SetSentConnected(int channel, bool connected)
        {
            if(IsValidChannel(channel))
            {
                transmitters[channel].SetConnected(connected);
            }
        }

        public GPIO Channel01IRQ { get; }
        public GPIO Channel23IRQ { get; }
        public GPIO Channel45IRQ { get; }
        public GPIO Channel67IRQ { get; }
        public GPIO OverflowIRQ { get; }

        public long Size => 0x1000;

        private bool IsValidChannel(int channel)
        {
            if(channel >= 0 && channel < NumberOfChannels)
            {
                return true;
            }
            this.Log(LogLevel.Warning, "Channel {0} out of range (0..{1})", channel, NumberOfChannels - 1);
            return false;
        }

        /// <summary>The machine's virtual time in ns, the time base of all edges.</summary>
        private double NowNanoseconds => machine.ClockSource.CurrentValue.Ticks / (double)TimeInterval.TicksPerNanosecond;

        /// <summary>The counter value (CNT) at a point of virtual time since the counter was started.</summary>
        private uint CounterAt(double timeNanoseconds)
        {
            if(counterFrequency == 0)
            {
                return (uint)counterInitialValue.Value;
            }
            var range = (ulong)Math.Max(1, ((long)modulo.Value - (long)counterInitialValue.Value & 0xFFFF) + 1);
            var counts = (ulong)Math.Max(0.0, Math.Floor((timeNanoseconds - counterStartNanoseconds) * counterFrequency / 1e9));
            return (uint)((counterInitialValue.Value + counts % range) & 0xFFFF);
        }

        private uint ReadCounter()
        {
            // An edge waiting to be handled: the interrupt handler reading CNT would have run right at it
            var pending = Enumerable.Range(0, NumberOfChannels).FirstOrDefault(channel => pendingCapture[channel].HasValue && channelInterruptEnable[channel].Value, -1);
            if(pending >= 0)
            {
                return pendingCapture[pending].Value;
            }
            if(machine.GetSystemBus(this).TryGetCurrentCPU(out var cpu))
            {
                cpu.SyncTime();
            }
            return CounterAt(NowNanoseconds);
        }

        private void UpdateCounter()
        {
            var source = clockSource.Value;
            var frequency = source == ClockSourceFixedFrequency ? fixedFrequencyClock
                : source == ClockSourceInput ? inputClock
                : 0UL;
            if(source == ClockSourceExternal)
            {
                this.Log(LogLevel.Warning, "External clock source is not supported, the counter stops");
            }

            counterStartNanoseconds = NowNanoseconds;
            if(frequency == 0)
            {
                counterFrequency = 0;
                counter.Enabled = false;
                return;
            }
            counterFrequency = (double)frequency / (1UL << (int)prescaler.Value);
            counter.Frequency = frequency;
            counter.Divider = 1UL << (int)prescaler.Value;
            counter.Limit = Math.Max(1UL, ((modulo.Value - counterInitialValue.Value) & 0xFFFF) + 1);
            counter.Value = 0;
            counter.Enabled = true;
        }

        private void OnCounterOverflow()
        {
            timerOverflow.Value = true;
            UpdateInterrupts();
        }

        private bool IsCapturingFallingEdges(int channel)
        {
            // Input capture mode (MSnB:MSnA = 00) on falling (ELSnB:ELSnA = 10) or both (11) edges
            return modeSelect[channel].Value == 0 && (edgeSelect[channel].Value & 0b10) != 0;
        }

        /// <summary>Hands the channel's next due edge to the firmware once it has handled the previous one.</summary>
        private void DeliverNextEdge(int channel)
        {
            if(channelFlag[channel].Value || !transmitters[channel].TryTakeEdge(out var edgeNanoseconds))
            {
                return;
            }
            if(!IsCapturingFallingEdges(channel))
            {
                return;
            }
            var capture = CounterAt(edgeNanoseconds);
            channelValue[channel].Value = capture;
            pendingCapture[channel] = capture;
            channelFlag[channel].Value = true;
            UpdateInterrupts();
        }

        private void OnChannelFlagCleared(int channel)
        {
            pendingCapture[channel] = null;
            DeliverNextEdge(channel);
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var pairs = new[] { Channel01IRQ, Channel23IRQ, Channel45IRQ, Channel67IRQ };
            for(var pair = 0; pair < pairs.Length; pair++)
            {
                var pending = Enumerable.Range(pair * 2, 2).Any(channel => channelFlag[channel].Value && channelInterruptEnable[channel].Value);
                pairs[pair].Set(pending);
            }
            OverflowIRQ.Set(timerOverflow.Value && timerOverflowInterruptEnable.Value);
        }

        private void DefineRegisters()
        {
            Registers.StatusAndControl.Define(this)
                .WithValueField(0, 3, out prescaler, name: "PS", writeCallback: (_, __) => UpdateCounter())
                .WithValueField(3, 2, out clockSource, name: "CLKS", writeCallback: (_, __) => UpdateCounter())
                .WithFlag(5, name: "CPWMS")
                .WithFlag(6, name: "RIE")
                .WithFlag(7, name: "RF")
                .WithFlag(8, out timerOverflowInterruptEnable, name: "TOIE", writeCallback: (_, __) => UpdateInterrupts())
                .WithFlag(9, out timerOverflow, FieldMode.Read | FieldMode.WriteZeroToClear, name: "TOF", writeCallback: (_, __) => UpdateInterrupts())
                .WithReservedBits(10, 6)
                .WithValueField(16, 8, name: "PWMEN")
                .WithValueField(24, 8, name: "FLTPS");

            Registers.Counter.Define(this)
                .WithValueField(0, 16, name: "COUNT",
                    valueProviderCallback: _ => ReadCounter(),
                    // Writing any value resets the counter to CNTIN
                    writeCallback: (_, __) => UpdateCounter())
                .WithReservedBits(16, 16);

            Registers.Modulo.Define(this, 0xFFFF)
                .WithValueField(0, 16, out modulo, name: "MOD", writeCallback: (_, __) => UpdateCounter())
                .WithReservedBits(16, 16);

            for(var channel = 0; channel < NumberOfChannels; channel++)
            {
                var index = channel;
                ((Registers)((long)Registers.Channel0StatusAndControl + index * ChannelRegistersStride)).Define(this)
                    .WithFlag(0, name: "DMA")
                    .WithFlag(1, name: "ICRST")
                    .WithValueField(2, 2, out edgeSelect[index], name: "ELS")
                    .WithValueField(4, 2, out modeSelect[index], name: "MS")
                    .WithFlag(6, out channelInterruptEnable[index], name: "CHIE", writeCallback: (previous, enabled) =>
                    {
                        if(enabled && !previous)
                        {
                            transmitters[index].Start();
                            DeliverNextEdge(index);
                        }
                        else if(!enabled && previous)
                        {
                            transmitters[index].Stop();
                        }
                        UpdateInterrupts();
                    })
                    .WithFlag(7, out channelFlag[index], FieldMode.Read | FieldMode.WriteZeroToClear, name: "CHF",
                        changeCallback: (_, value) =>
                        {
                            if(!value)
                            {
                                OnChannelFlagCleared(index);
                            }
                        })
                    .WithValueField(8, 24, name: "CnSC_upper");

                ((Registers)((long)Registers.Channel0Value + index * ChannelRegistersStride)).Define(this)
                    .WithValueField(0, 16, out channelValue[index], name: "VAL")
                    .WithReservedBits(16, 16);
            }

            Registers.CounterInitialValue.Define(this)
                .WithValueField(0, 16, out counterInitialValue, name: "INIT", writeCallback: (_, __) => UpdateCounter())
                .WithReservedBits(16, 16);

            Registers.Status.Define(this)
                .WithFlags(0, NumberOfChannels, name: "CHnF",
                    valueProviderCallback: (channel, _) => channelFlag[channel].Value,
                    writeCallback: (channel, _, value) =>
                    {
                        if(!value && channelFlag[channel].Value)
                        {
                            channelFlag[channel].Value = false;
                            OnChannelFlagCleared(channel);
                        }
                    })
                .WithReservedBits(NumberOfChannels, 32 - NumberOfChannels);

            // Configuration of the features that aren't modelled, kept so the firmware reads back what it wrote
            for(var offset = (long)Registers.Mode; offset <= (long)Registers.Pair3Deadtime; offset += 4)
            {
                RegistersCollection.AddRegister(offset, new DoubleWordRegister(this).WithValueField(0, 32, name: "storage"));
            }
        }

        private readonly ulong fixedFrequencyClock;
        private readonly ulong inputClock;
        private readonly LimitTimer counter;
        private readonly SentTransmitter[] transmitters;
        private readonly uint?[] pendingCapture;
        private double counterFrequency;
        private double counterStartNanoseconds;

        private readonly IFlagRegisterField[] channelFlag;
        private readonly IFlagRegisterField[] channelInterruptEnable;
        private readonly IValueRegisterField[] modeSelect;
        private readonly IValueRegisterField[] edgeSelect;
        private readonly IValueRegisterField[] channelValue;
        private IValueRegisterField prescaler;
        private IValueRegisterField clockSource;
        private IValueRegisterField modulo;
        private IValueRegisterField counterInitialValue;
        private IFlagRegisterField timerOverflow;
        private IFlagRegisterField timerOverflowInterruptEnable;

        private const int NumberOfChannels = 8;
        private const long ChannelRegistersStride = 0x8;
        private const ulong ClockSourceInput = 0b01;
        private const ulong ClockSourceFixedFrequency = 0b10;
        private const ulong ClockSourceExternal = 0b11;

        /// <summary>
        /// A SENT sensor on one channel input. Computes its falling edges from the frame schedule and queues
        /// the ones that are due for the FTM to deliver.
        /// </summary>
        private class SentTransmitter
        {
            public SentTransmitter(S32K1XX_FTM owner, IMachine machine, int channel)
            {
                this.owner = owner;
                this.channel = channel;
                dueEdges = new Queue<double>();
                // Only wakes the transmitter up to queue due edges; when exactly it runs doesn't matter
                wakeUp = new LimitTimer(machine.ClockSource, 1000000000, owner, $"sent{channel}", limit: 1,
                    direction: Direction.Ascending, enabled: false, workMode: WorkMode.Periodic, eventEnabled: true);
                wakeUp.LimitReached += () =>
                {
                    QueueDueEdges();
                    owner.DeliverNextEdge(channel);
                };
            }

            public void SetFrame(uint[] nibbles, double tickMicroseconds)
            {
                var tickNanoseconds = Math.Max(1.0, tickMicroseconds * 1000.0);
                // Time between falling edges: the sync pulse, then 12 + value ticks per nibble
                var intervals = new[] { SyncPulseTicks }.Concat(nibbles.Select(nibble => ZeroNibbleTicks + nibble))
                    .Select(ticks => ticks * tickNanoseconds).ToArray();
                if(this.intervals == null)
                {
                    this.intervals = intervals;
                    if(listening && connected)
                    {
                        Start();
                        owner.DeliverNextEdge(channel);
                    }
                }
                else
                {
                    // Takes over with the next sync pulse, so no frame mixes old and new nibbles
                    pendingIntervals = intervals;
                }
            }

            public void SetConnected(bool connected)
            {
                if(connected == this.connected)
                {
                    return;
                }
                if(!connected)
                {
                    // What was sent until now still arrives
                    QueueDueEdges();
                    wakeUp.Enabled = false;
                    sending = false;
                }
                this.connected = connected;
                if(connected && listening)
                {
                    Start();
                    owner.DeliverNextEdge(channel);
                }
            }

            /// <summary>The receiver starts listening: a falling edge now, then the frame from its sync pulse on.</summary>
            public void Start()
            {
                listening = true;
                dueEdges.Clear();
                if(!connected || intervals == null)
                {
                    return;
                }
                if(pendingIntervals != null)
                {
                    intervals = pendingIntervals;
                    pendingIntervals = null;
                }
                var now = owner.NowNanoseconds;
                position = 0;
                nextEdgeNanoseconds = now + intervals[0];
                sending = true;
                dueEdges.Enqueue(now);
                ScheduleWakeUp(now);
            }

            public void Stop()
            {
                listening = false;
                sending = false;
                wakeUp.Enabled = false;
                dueEdges.Clear();
            }

            public bool TryTakeEdge(out double edgeNanoseconds)
            {
                QueueDueEdges();
                if(dueEdges.Count > 0)
                {
                    edgeNanoseconds = dueEdges.Dequeue();
                    return true;
                }
                edgeNanoseconds = 0;
                return false;
            }

            private void QueueDueEdges()
            {
                if(!sending)
                {
                    return;
                }
                var now = owner.NowNanoseconds;
                while(nextEdgeNanoseconds <= now)
                {
                    dueEdges.Enqueue(nextEdgeNanoseconds);
                    position = (position + 1) % intervals.Length;
                    if(position == 0 && pendingIntervals != null)
                    {
                        intervals = pendingIntervals;
                        pendingIntervals = null;
                    }
                    nextEdgeNanoseconds += intervals[position];
                }
                ScheduleWakeUp(now);
            }

            private void ScheduleWakeUp(double now)
            {
                // Periodic, so rescheduling from within its own event isn't undone when the event ends
                wakeUp.Value = 0;
                wakeUp.Limit = (ulong)Math.Max(1.0, Math.Ceiling(nextEdgeNanoseconds - now));
                wakeUp.Enabled = true;
            }

            private readonly S32K1XX_FTM owner;
            private readonly int channel;
            private readonly LimitTimer wakeUp;
            private readonly Queue<double> dueEdges;
            private double[] intervals;
            private double[] pendingIntervals;
            private double nextEdgeNanoseconds;
            private int position;
            private bool connected = true;
            private bool listening;
            private bool sending;

            private const uint SyncPulseTicks = 56;
            private const uint ZeroNibbleTicks = 12;
        }

        private enum Registers : long
        {
            StatusAndControl = 0x00,
            Counter = 0x04,
            Modulo = 0x08,
            Channel0StatusAndControl = 0x0C,
            Channel0Value = 0x10,
            CounterInitialValue = 0x4C,
            Status = 0x50,
            Mode = 0x54,
            Pair3Deadtime = 0xB8,
        }
    }
}
