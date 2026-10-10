//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Video
{
    /// <summary>
    /// Sitronix ST7789 TFT display controller on a 4-line SPI interface, headless: it keeps the frame
    /// memory the MCU writes, so that it can be read back from outside (GetPixel, GetRgb888).
    ///
    /// GPIO inputs: 0 = D/CX (low: the byte is a command, high: a parameter or pixel data), 1 = RESX
    /// (active low hardware reset).
    ///
    /// The frame memory is kept in the MCU's coordinates: x is the column the MCU sets with CASET, y the
    /// row it sets with RASET. MADCTL's row/column exchange (MV) swaps the 240 x 320 memory to 320 x 240,
    /// its mirroring bits only change how the panel shows the memory and are not applied - the picture is
    /// what the MCU drew, the way it meant to draw it. Pixels are kept as RGB565, 18-bit pixels are reduced
    /// to that. Display inversion (INVON) isn't applied either: IPS panels need it to show the colours
    /// the MCU writes. Timing (sleep out delay, SPI clock) isn't modelled, reads (RDDID etc.) return 0.
    /// </summary>
    public class ST7789 : ISPIPeripheral, IGPIOReceiver
    {
        public ST7789(IMachine machine)
        {
            frame = new ushort[MaxSide * MaxSide];
            parameters = new byte[4];
            Reset();
        }

        public void Reset()
        {
            System.Array.Clear(frame, 0, frame.Length);
            dataMode = false;
            command = 0;
            SoftwareReset();
        }

        public void OnGPIO(int number, bool value)
        {
            switch(number)
            {
            case DataCommandInput:
                dataMode = value;
                break;
            case ResetInput:
                if(!value)
                {
                    SoftwareReset();
                }
                break;
            default:
                this.Log(LogLevel.Warning, "Unknown GPIO input {0}", number);
                break;
            }
        }

        public byte Transmit(byte data)
        {
            if(!dataMode)
            {
                StartCommand(data);
            }
            else
            {
                WriteParameter(data);
            }
            return 0;
        }

        public void FinishTransmission()
        {
            // The command stays active across chip select: parameters and pixel data may follow in
            // further transfers, as long as D/CX is high
        }

        /// <summary>RGB565 value of a pixel, in the MCU's coordinates.</summary>
        public ushort GetPixel(int x, int y)
        {
            return (x >= 0 && x < Width && y >= 0 && y < Height) ? frame[y * MaxSide + x] : (ushort)0;
        }

        /// <summary>The whole picture, Width x Height pixels row by row, 3 bytes (R, G, B) each.</summary>
        public byte[] GetRgb888()
        {
            var width = Width;
            var height = Height;
            var result = new byte[width * height * 3];
            var i = 0;
            for(var y = 0; y < height; y++)
            {
                for(var x = 0; x < width; x++)
                {
                    var pixel = frame[y * MaxSide + x];
                    var r = (pixel >> 11) & 0x1F;
                    var g = (pixel >> 5) & 0x3F;
                    var b = pixel & 0x1F;
                    result[i++] = (byte)((r << 3) | (r >> 2));
                    result[i++] = (byte)((g << 2) | (g >> 4));
                    result[i++] = (byte)((b << 3) | (b >> 2));
                }
            }
            return result;
        }

        public int Width => RowColumnExchange ? MaxSide : MinSide;

        public int Height => RowColumnExchange ? MinSide : MaxSide;

        public bool Sleeping { get; private set; }

        public bool DisplayOn { get; private set; }

        public bool Inverted { get; private set; }

        /// <summary>Number of pixels written since the start, e.g. to see whether the MCU still draws.</summary>
        public ulong PixelsWritten { get; private set; }

        private bool RowColumnExchange => (memoryAccessControl & MadctlRowColumnExchange) != 0;

        private void SoftwareReset()
        {
            Sleeping = true;
            DisplayOn = false;
            Inverted = false;
            memoryAccessControl = 0;
            pixelFormat = DefaultPixelFormat;
            columnStart = 0;
            columnEnd = MinSide - 1;
            rowStart = 0;
            rowEnd = MaxSide - 1;
            column = 0;
            row = 0;
            parameterCount = 0;
            pixelByteCount = 0;
        }

        private void StartCommand(byte data)
        {
            command = (Command)data;
            parameterCount = 0;

            switch(command)
            {
            case Command.SoftwareReset:
                SoftwareReset();
                break;
            case Command.SleepIn:
                Sleeping = true;
                break;
            case Command.SleepOut:
                Sleeping = false;
                break;
            case Command.InversionOff:
                Inverted = false;
                break;
            case Command.InversionOn:
                Inverted = true;
                break;
            case Command.DisplayOff:
                DisplayOn = false;
                break;
            case Command.DisplayOn:
                DisplayOn = true;
                break;
            case Command.MemoryWrite:
                column = columnStart;
                row = rowStart;
                pixelByteCount = 0;
                break;
            case Command.MemoryWriteContinue:
                pixelByteCount = 0;
                break;
            }
        }

        private void WriteParameter(byte data)
        {
            switch(command)
            {
            case Command.ColumnAddressSet:
            case Command.RowAddressSet:
                if(parameterCount < parameters.Length)
                {
                    parameters[parameterCount++] = data;
                }
                if(parameterCount == parameters.Length)
                {
                    var start = (parameters[0] << 8) | parameters[1];
                    var end = (parameters[2] << 8) | parameters[3];
                    if(command == Command.ColumnAddressSet)
                    {
                        columnStart = start;
                        columnEnd = end;
                    }
                    else
                    {
                        rowStart = start;
                        rowEnd = end;
                    }
                }
                break;
            case Command.MemoryAccessControl:
                memoryAccessControl = data;
                break;
            case Command.InterfacePixelFormat:
                pixelFormat = data;
                break;
            case Command.MemoryWrite:
            case Command.MemoryWriteContinue:
                WritePixelByte(data);
                break;
            default:
                // Parameters of commands that don't change the picture (gamma, porch, power settings, ...)
                break;
            }
        }

        private void WritePixelByte(byte data)
        {
            pixelBytes[pixelByteCount++] = data;
            var colourFormat = pixelFormat & 0x7;
            ushort pixel;
            if(colourFormat == ColourFormat18Bit)
            {
                if(pixelByteCount < 3)
                {
                    return;
                }
                // 6 bits per colour in the upper bits of each byte
                pixel = (ushort)(((pixelBytes[0] >> 3) << 11) | ((pixelBytes[1] >> 2) << 5) | (pixelBytes[2] >> 3));
            }
            else
            {
                if(colourFormat != ColourFormat16Bit && !warnedAboutPixelFormat)
                {
                    this.Log(LogLevel.Warning, "Pixel format 0x{0:X2} isn't supported, treating it as 16 bit", pixelFormat);
                    warnedAboutPixelFormat = true;
                }
                if(pixelByteCount < 2)
                {
                    return;
                }
                pixel = (ushort)((pixelBytes[0] << 8) | pixelBytes[1]);
            }
            pixelByteCount = 0;

            if(column < Width && row < Height)
            {
                frame[row * MaxSide + column] = pixel;
            }
            PixelsWritten++;

            // Within the window set by CASET/RASET, row by row, wrapping around at its end
            column++;
            if(column > columnEnd)
            {
                column = columnStart;
                row++;
                if(row > rowEnd)
                {
                    row = rowStart;
                }
            }
        }

        private readonly ushort[] frame;
        private readonly byte[] parameters;
        private readonly byte[] pixelBytes = new byte[3];

        private bool dataMode;
        private Command command;
        private int parameterCount;
        private int pixelByteCount;
        private byte memoryAccessControl;
        private byte pixelFormat;
        private int columnStart;
        private int columnEnd;
        private int rowStart;
        private int rowEnd;
        private int column;
        private int row;
        private bool warnedAboutPixelFormat;

        private const int DataCommandInput = 0;
        private const int ResetInput = 1;
        private const int MinSide = 240;
        private const int MaxSide = 320;
        private const byte MadctlRowColumnExchange = 1 << 5;
        private const byte DefaultPixelFormat = 0x66;
        private const int ColourFormat16Bit = 0x5;
        private const int ColourFormat18Bit = 0x6;

        private enum Command : byte
        {
            SoftwareReset = 0x01,
            SleepIn = 0x10,
            SleepOut = 0x11,
            InversionOff = 0x20,
            InversionOn = 0x21,
            DisplayOff = 0x28,
            DisplayOn = 0x29,
            ColumnAddressSet = 0x2A,
            RowAddressSet = 0x2B,
            MemoryWrite = 0x2C,
            MemoryAccessControl = 0x36,
            MemoryWriteContinue = 0x3C,
            InterfacePixelFormat = 0x3A,
        }
    }
}
