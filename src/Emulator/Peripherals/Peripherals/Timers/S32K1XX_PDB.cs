//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Analog;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Timers
{
    /// <summary>
    /// S32K1xx Programmable Delay Block, modelled as far as it triggers ADC conversions.
    ///
    /// A software trigger (SC.SWTRIG with SC.TRGSEL = software and SC.PDBEN set) immediately fires every
    /// enabled pretrigger, in order, into the connected ADC: pretrigger m of channel n triggers ADC slot
    /// n * 8 + m. Counter, delays, continuous mode and the PDB's own interrupts are not modelled; their
    /// registers are plain storage so that drivers can configure them.
    /// </summary>
    public class S32K1XX_PDB : BasicDoubleWordPeripheral, IKnownSize
    {
        public S32K1XX_PDB(IMachine machine, S32K1XX_ADC adc) : base(machine)
        {
            this.adc = adc;
            pretriggerEnable = new IValueRegisterField[NumberOfChannels];
            DefineRegisters();
        }

        public long Size => 0x1000;

        private void SoftwareTrigger()
        {
            if(!enabled.Value)
            {
                this.Log(LogLevel.Warning, "Software trigger while the PDB is disabled");
                return;
            }
            if(triggerSelect.Value != SoftwareTriggerSelect)
            {
                this.Log(LogLevel.Warning, "Software trigger while TRGSEL selects hardware input {0}", triggerSelect.Value);
                return;
            }

            for(var channel = 0; channel < NumberOfChannels; channel++)
            {
                for(var pretrigger = 0; pretrigger < PretriggersPerChannel; pretrigger++)
                {
                    if((pretriggerEnable[channel].Value & (1UL << pretrigger)) != 0)
                    {
                        adc.TriggerSlot(channel * PretriggersPerChannel + pretrigger);
                    }
                }
            }
        }

        private void DefineRegisters()
        {
            Registers.StatusAndControl.Define(this)
                // Values are loaded immediately, so LDOK never stays set
                .WithFlag(0, FieldMode.Read | FieldMode.Set, valueProviderCallback: _ => false, name: "LDOK")
                .WithFlag(1, name: "CONT")
                .WithValueField(2, 2, name: "MULT")
                .WithReservedBits(4, 1)
                .WithFlag(5, name: "PDBIE")
                .WithFlag(6, FieldMode.Read | FieldMode.WriteZeroToClear, name: "PDBIF")
                .WithFlag(7, out enabled, name: "PDBEN")
                .WithValueField(8, 4, out triggerSelect, name: "TRGSEL")
                .WithValueField(12, 3, name: "PRESCALER")
                .WithFlag(15, name: "DMAEN")
                .WithFlag(16, FieldMode.Read | FieldMode.Set, valueProviderCallback: _ => false, writeCallback: (_, value) =>
                {
                    if(value)
                    {
                        SoftwareTrigger();
                    }
                }, name: "SWTRIG")
                .WithFlag(17, name: "PDBEIE")
                .WithValueField(18, 2, name: "LDMOD")
                .WithReservedBits(20, 12);

            Registers.Modulus.Define(this, resetValue: 0xFFFF)
                .WithValueField(0, 16, name: "MOD")
                .WithReservedBits(16, 16);

            Registers.Counter.Define(this)
                .WithValueField(0, 16, FieldMode.Read, valueProviderCallback: _ => 0, name: "CNT")
                .WithReservedBits(16, 16);

            Registers.InterruptDelay.Define(this, resetValue: 0xFFFF)
                .WithValueField(0, 16, name: "IDLY")
                .WithReservedBits(16, 16);

            for(var channel = 0; channel < NumberOfChannels; channel++)
            {
                var channelBase = (long)Registers.Channel0Control1 + channel * ChannelStride;
                RegistersCollection.AddRegister(channelBase, new DoubleWordRegister(this)
                    .WithValueField(0, 8, out pretriggerEnable[channel], name: "EN")
                    .WithValueField(8, 8, name: "TOS")
                    .WithValueField(16, 8, name: "BB")
                    .WithReservedBits(24, 8));
                // CHnS: no sequence errors or delay events happen without a modelled counter
                RegistersCollection.AddRegister(channelBase + 0x4, new DoubleWordRegister(this)
                    .WithValueField(0, 8, FieldMode.Read | FieldMode.WriteOneToClear, name: "ERR")
                    .WithReservedBits(8, 8)
                    .WithValueField(16, 8, FieldMode.Read | FieldMode.WriteZeroToClear, name: "CF")
                    .WithReservedBits(24, 8));
                for(var delay = 0; delay < PretriggersPerChannel; delay++)
                {
                    RegistersCollection.AddRegister(channelBase + 0x8 + delay * 4, new DoubleWordRegister(this)
                        .WithValueField(0, 16, name: "DLY")
                        .WithReservedBits(16, 16));
                }
            }

            Registers.PulseOutEnable.Define(this)
                .WithValueField(0, 8, name: "POEN")
                .WithReservedBits(8, 24);

            Registers.PulseOutDelay.Define(this)
                .WithValueField(0, 16, name: "DLY2")
                .WithValueField(16, 16, name: "DLY1");
        }

        private IFlagRegisterField enabled;
        private IValueRegisterField triggerSelect;

        private readonly IValueRegisterField[] pretriggerEnable;
        private readonly S32K1XX_ADC adc;

        private const int NumberOfChannels = 2;
        private const int PretriggersPerChannel = 8;
        private const int ChannelStride = 0x28;
        private const ulong SoftwareTriggerSelect = 0xF;

        private enum Registers
        {
            StatusAndControl = 0x00,  // SC
            Modulus = 0x04,           // MOD
            Counter = 0x08,           // CNT
            InterruptDelay = 0x0C,    // IDLY
            Channel0Control1 = 0x10,  // CH0C1, then CH0S, CH0DLY0..7; CH1 at +0x28
            PulseOutEnable = 0x190,   // POEN
            PulseOutDelay = 0x194,    // PO0DLY
        }
    }
}
