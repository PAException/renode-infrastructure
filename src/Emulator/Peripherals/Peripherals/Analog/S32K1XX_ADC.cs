//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Analog
{
    /// <summary>
    /// S32K1xx 12-bit SAR ADC.
    ///
    /// Conversions complete immediately. They are started by a write to SC1A in software trigger mode
    /// (SC2.ADTRG = 0), or per slot by a pretrigger from the PDB (see S32K1XX_PDB) in hardware trigger mode.
    /// The analog inputs are set from outside as raw conversion results per input channel (SetChannelValue),
    /// the converted slot picks the channel via its SC1n.ADCH like on the real chip.
    /// Calibration and averaging finish immediately, offset/gain correction is not modelled.
    /// </summary>
    public class S32K1XX_ADC : BasicDoubleWordPeripheral, IKnownSize
    {
        public S32K1XX_ADC(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            inputChannel = new IValueRegisterField[NumberOfSlots];
            interruptEnable = new IFlagRegisterField[NumberOfSlots];
            conversionComplete = new IFlagRegisterField[NumberOfSlots];
            result = new IValueRegisterField[NumberOfSlots];
            channelValues = new uint[NumberOfChannels];
            DefineRegisters();
        }

        public override void Reset()
        {
            base.Reset();
            UpdateInterrupts();
        }

        /// <summary>Sets the raw value (0..4095) the given input channel (ADCH) converts to.</summary>
        public void SetChannelValue(int channel, uint value)
        {
            if(channel < 0 || channel >= NumberOfChannels)
            {
                this.Log(LogLevel.Warning, "Input channel {0} out of range (0..{1})", channel, NumberOfChannels - 1);
                return;
            }
            channelValues[channel] = value & ResultMask;
        }

        public uint GetChannelValue(int channel) => channelValues[channel];

        /// <summary>Hardware trigger of one conversion slot, as raised by the PDB's pretrigger for that slot.</summary>
        public void TriggerSlot(int slot)
        {
            if(slot < 0 || slot >= NumberOfSlots)
            {
                this.Log(LogLevel.Warning, "Trigger for nonexistent slot {0}", slot);
                return;
            }
            if(!hardwareTrigger.Value)
            {
                this.Log(LogLevel.Debug, "Ignoring hardware trigger for slot {0}, software trigger mode is selected", slot);
                return;
            }
            Convert(slot);
        }

        public GPIO IRQ { get; }

        public long Size => 0x1000;

        private void Convert(int slot)
        {
            var channel = (int)inputChannel[slot].Value;
            if(channel == ModuleDisabledChannel)
            {
                return;
            }
            result[slot].Value = channelValues[channel];
            conversionComplete[slot].Value = true;
            this.Log(LogLevel.Noisy, "Slot {0} converted channel {1}: {2}", slot, channel, result[slot].Value);
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var irq = Enumerable.Range(0, NumberOfSlots).Any(i => conversionComplete[i].Value && interruptEnable[i].Value);
            IRQ.Set(irq);
        }

        private void DefineRegisters()
        {
            Registers.StatusAndControl1.DefineMany(this, (uint)NumberOfSlots, (register, idx) =>
            {
                register
                    .WithValueField(0, 5, out inputChannel[idx], name: "ADCH")
                    .WithReservedBits(5, 1)
                    .WithFlag(6, out interruptEnable[idx], name: "AIEN")
                    .WithFlag(7, out conversionComplete[idx], FieldMode.Read, name: "COCO")
                    .WithReservedBits(8, 24)
                    .WithWriteCallback((_, __) =>
                    {
                        // A write aborts the slot's conversion and clears its result flag
                        conversionComplete[idx].Value = false;
                        // Only SC1A starts a conversion, and only in software trigger mode
                        if(idx == 0 && !hardwareTrigger.Value)
                        {
                            Convert(0);
                        }
                        UpdateInterrupts();
                    });
            }, resetValue: (uint)ModuleDisabledChannel);

            Registers.Configuration1.Define(this)
                .WithValueField(0, 2, name: "ADICLK")
                .WithValueField(2, 2, name: "MODE")
                .WithReservedBits(4, 1)
                .WithValueField(5, 2, name: "ADIV")
                .WithReservedBits(7, 1)
                .WithFlag(8, name: "CLRLTRG")
                .WithReservedBits(9, 23);

            Registers.Configuration2.Define(this, resetValue: 0x0000000C)
                .WithValueField(0, 8, name: "SMPLTS")
                .WithReservedBits(8, 24);

            Registers.DataResult.DefineMany(this, (uint)NumberOfSlots, (register, idx) =>
            {
                register
                    .WithValueField(0, 12, out result[idx], FieldMode.Read, name: "D")
                    .WithReservedBits(12, 20)
                    .WithReadCallback((_, __) =>
                    {
                        // Reading the result clears the slot's COCO and with it the interrupt request
                        conversionComplete[idx].Value = false;
                        UpdateInterrupts();
                    });
            });

            Registers.CompareValue.DefineMany(this, (uint)NumberOfCompareValues, (register, idx) =>
            {
                register
                    .WithValueField(0, 16, name: "CV")
                    .WithReservedBits(16, 16);
            });

            Registers.StatusAndControl2.Define(this)
                .WithValueField(0, 2, name: "REFSEL")
                .WithFlag(2, name: "DMAEN")
                .WithFlag(3, name: "ACREN")
                .WithFlag(4, name: "ACFGT")
                .WithFlag(5, name: "ACFE")
                .WithFlag(6, out hardwareTrigger, name: "ADTRG")
                // Conversions complete immediately, so none is ever active
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => false, name: "ADACT")
                .WithReservedBits(8, 4)
                .WithValueField(12, 4, FieldMode.Read, valueProviderCallback: _ => 0, name: "TRGPRNUM")
                .WithValueField(16, 4, FieldMode.Read, valueProviderCallback: _ => 0, name: "TRGSTLAT")
                .WithReservedBits(20, 4)
                .WithValueField(24, 4, FieldMode.Read, valueProviderCallback: _ => 0, name: "TRGSTERR")
                .WithReservedBits(28, 4);

            Registers.StatusAndControl3.Define(this)
                .WithValueField(0, 2, name: "AVGS")
                .WithFlag(2, name: "AVGE")
                .WithFlag(3, name: "ADCO")
                .WithReservedBits(4, 3)
                // Calibration finishes immediately
                .WithFlag(7, valueProviderCallback: _ => false, name: "CAL")
                .WithReservedBits(8, 24);

            // Offset, gain and calibration value registers: plain storage, the conversion is ideal
            for(var offset = (long)Registers.BaseOffset; offset <= (long)Registers.CalibrationOffset9; offset += 4)
            {
                RegistersCollection.AddRegister(offset, new DoubleWordRegister(this).WithValueField(0, 32, name: "CALIBRATION_DATA"));
            }
        }

        private IFlagRegisterField hardwareTrigger;

        private readonly IValueRegisterField[] inputChannel;
        private readonly IFlagRegisterField[] interruptEnable;
        private readonly IFlagRegisterField[] conversionComplete;
        private readonly IValueRegisterField[] result;
        private readonly uint[] channelValues;

        private const int NumberOfSlots = 16;
        private const int NumberOfChannels = 32;
        private const int NumberOfCompareValues = 2;
        private const uint ResultMask = 0xFFF;
        // ADCH = 0x1F disables the module (reset value)
        private const int ModuleDisabledChannel = 0x1F;

        private enum Registers
        {
            StatusAndControl1 = 0x00,   // SC1A..SC1P
            Configuration1 = 0x40,      // CFG1
            Configuration2 = 0x44,      // CFG2
            DataResult = 0x48,          // RA..RP
            CompareValue = 0x88,        // CV1, CV2
            StatusAndControl2 = 0x90,   // SC2
            StatusAndControl3 = 0x94,   // SC3
            BaseOffset = 0x98,          // BASE_OFS
            CalibrationOffset9 = 0xE8,  // CLP9_OFS
        }
    }
}
