using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;

namespace ThVisualizer
{
    public class ThermalMap
    {
#pragma warning disable 649
#pragma warning disable 169
        struct ThmHeader
        {
            public unsafe fixed byte signature[4];
            public UInt16 width;
            public UInt16 height;
        };
#pragma warning restore 169
#pragma warning restore 649

        private int width;
        private int height;
        private List<float> thermalMap = new List<float>();
        private List<UInt16> colorMap = new List<ushort>();

        private static readonly byte[] DEFAULT_COLOR_SCHEME = new byte[] {
            28, 1, 108,
            31, 17, 218 ,
            50, 111, 238 ,
            63, 196, 229 ,
            64, 222, 135 ,
            192, 240, 14 ,
            223, 172, 18 ,
            209, 111, 14 ,
            210, 50, 28 ,
            194, 26, 0 ,
            132, 26, 0 
        };

        public ThermalMap()
        {

        }

        public ThermalMap(String fileName)
        {
            using (var reader = new BinaryReader(File.OpenRead(fileName)))
            {
                try
                {
                    byte[] headerBytes = reader.ReadBytes(Marshal.SizeOf(typeof (ThmHeader)));
                    GCHandle handle = GCHandle.Alloc(headerBytes, GCHandleType.Pinned);
                    ThmHeader header = (ThmHeader) Marshal.PtrToStructure(handle.AddrOfPinnedObject(), typeof (ThmHeader));
                    handle.Free();
                    width = header.width;
                    height = header.height;

                    float[] thermalData = new float[width*height];

                    if ((width > 0) && (height > 0))
                    {
                        var thermalBytes = reader.ReadBytes(width*height*sizeof(float));
                        Buffer.BlockCopy(thermalBytes, 0, thermalData, 0, thermalBytes.Count());
                        thermalMap.AddRange(thermalData.ToList());
                    }
                }
                catch (Exception ex)
                {
                    throw new Exception("Unable to read file "+ fileName + Environment.NewLine + ex);
                }
            }
        }

        public int Width
        {
            get { return width; }
        }

        public int Height
        {
            get { return height; }
        }

        public List<float> ThermMap
        {
            get { return thermalMap; }
        }

        private static ushort rgb2color(byte R, byte G, byte B)
        {
            return (ushort)(((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3));
        }

        private static byte calculateRGB(byte rgb1, byte rgb2, float t1, float step, float t) {
            return (byte)(rgb1 + (((t - t1) / step) * (rgb2 - rgb1)));
        }

        private static ushort temperatureToRGB565(float temperature, float minTemp, float maxTemp) {
            ushort val;
            if (temperature < minTemp) {
                val = rgb2color(DEFAULT_COLOR_SCHEME[0], DEFAULT_COLOR_SCHEME[1], DEFAULT_COLOR_SCHEME[2]);
            }
            else if (temperature >= maxTemp) {
                int colorSchemeSize = DEFAULT_COLOR_SCHEME.Length / 3;
                val = rgb2color(DEFAULT_COLOR_SCHEME[(colorSchemeSize - 1) * 3 + 0], DEFAULT_COLOR_SCHEME[(colorSchemeSize - 1) * 3 + 1], DEFAULT_COLOR_SCHEME[(colorSchemeSize - 1) * 3 + 2]);
            }
            else
            {
                float step = (maxTemp - minTemp) / 10.0f;
                byte step1 = (byte)((temperature - minTemp) / step);
                byte step2 = (byte)(step1 + 1);
                byte red = calculateRGB(DEFAULT_COLOR_SCHEME[step1 * 3 + 0], DEFAULT_COLOR_SCHEME[step2 * 3 + 0], (minTemp + step1 * step), step, temperature);
                byte green = calculateRGB(DEFAULT_COLOR_SCHEME[step1 * 3 + 1], DEFAULT_COLOR_SCHEME[step2 * 3 + 1], (minTemp + step1 * step), step, temperature);
                byte blue = calculateRGB(DEFAULT_COLOR_SCHEME[step1 * 3 + 2], DEFAULT_COLOR_SCHEME[step2 * 3 + 2], (minTemp + step1 * step), step, temperature);
                val = rgb2color(red, green, blue);
            }
            return val;
        }
    }
}