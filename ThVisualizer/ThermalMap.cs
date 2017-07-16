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
            public byte version;
            public UInt16 width;
            public UInt16 height;
            public byte isContainsTemperature;
            public byte isContainsThermalMap;
            public byte isContainsPhoto;
        };
#pragma warning restore 169
#pragma warning restore 649

        private int width;
        private int height;
        private bool isContainsTemperature;
        private bool isContainsThermalMap;
        private bool isContainsPhoto;
        private List<float> thermalMap = new List<float>();
        private List<UInt16> colorMap = new List<ushort>();
        private List<UInt16> photoMap = new List<ushort>();
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
                    isContainsTemperature = (header.isContainsTemperature != 0);
                    isContainsThermalMap = (header.isContainsThermalMap != 0);
                    isContainsPhoto = (header.isContainsPhoto != 0);

                    float[] thermalLine = new float[width];
                    UInt16[] colorLine = new ushort[width];
                    UInt16[] photoLine = new ushort[width];

                    if ((width > 0) && (height > 0))
                    {
                        if (isContainsPhoto)
                        {
                            for (int i = 0; i < height; i++)
                            {
                                try
                                {
                                    var photoLineBytes = reader.ReadBytes(width * sizeof(UInt16));
                                    Buffer.BlockCopy(photoLineBytes, 0, photoLine, 0, photoLineBytes.Count());
                                    photoMap.AddRange(photoLine.ToList());
                                }
                                catch (Exception)
                                {
                                    photoMap = new List<ushort>();
                                }
                            }
                        }
                        for (int i = 0; i < height; i++)
                        {
                            try
                            {
                                if (isContainsTemperature)
                                {
                                    var thermalLineBytes = reader.ReadBytes(width*sizeof (float));
                                    Buffer.BlockCopy(thermalLineBytes, 0, thermalLine, 0, thermalLineBytes.Count());
                                    thermalMap.AddRange(thermalLine.ToList());
                                }
                                if (isContainsThermalMap)
                                {
                                    var colorLineBytes = reader.ReadBytes(width*sizeof (UInt16));
                                    Buffer.BlockCopy(colorLineBytes, 0, colorLine, 0, colorLineBytes.Count());
                                    colorMap.AddRange(colorLine.ToList());
                                }
                            }
                            catch (Exception)
                            {
                                if (isContainsTemperature)
                                {
                                    thermalMap = new List<float>();
                                }
                                if (isContainsThermalMap)
                                {
                                    colorMap = new List<ushort>();
                                }
                            }
                        }
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

        public List<ushort> ColorMap
        {
            get { return colorMap; }
        }

        public List<ushort> PhotoMap
        {
            get { return photoMap; }
        }
    }
}