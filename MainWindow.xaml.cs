using System;
using System.Drawing;
using System.Linq;
using System.Windows;
using System.Windows.Media.Imaging;
using Color = System.Drawing.Color;
using PixelFormat = System.Drawing.Imaging.PixelFormat;
using Point = System.Windows.Point;

namespace ThVisualizer
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private int mapWidth = 320;
        private int mapHeight = 240;
        private float imageScale = 20;
        private ThermalMap thermalMap;


        public static Color[] DEFAULT_COLOR_SCHEME = new Color[]{
        Color.FromArgb(28, 1, 108),
        Color.FromArgb(31, 17, 218),
        Color.FromArgb(50, 111, 238),
        Color.FromArgb(63, 196, 229),
        Color.FromArgb(64, 222, 135),
        Color.FromArgb(192, 240, 14),
        Color.FromArgb(223, 172, 18),
        Color.FromArgb(209, 111, 14),
        Color.FromArgb(210, 50, 28),
        Color.FromArgb(194, 26, 0),
        Color.FromArgb(132, 26, 0)
        };

        public static Color[] ALTERNATE_COLOR_SCHEME = new Color[]{
        Color.FromArgb(0, 0, 5),
        Color.FromArgb(7, 1, 97),
        Color.FromArgb(51, 1, 194),
        Color.FromArgb(110, 2, 212),
        Color.FromArgb(158, 6, 150),
        Color.FromArgb(197, 30, 58),
        Color.FromArgb(218, 66, 0),
        Color.FromArgb(237, 137, 0),
        Color.FromArgb(246, 199, 23),
        Color.FromArgb(251, 248, 117),
        Color.FromArgb(252, 254, 253)
        };

        public MainWindow()
        {
            InitializeComponent();

            string[] args = Environment.GetCommandLineArgs();
            if (args.Count() > 1)
            {
                parseFile(args[1]);
                drawThermalMap(imageScale);
            }
        }

        private void btnQuit_Click(object sender, RoutedEventArgs e)
        {
            Application.Current.Shutdown();
        }

        private void btnOpenFile_Click(object sender, RoutedEventArgs e)
        {
            lblFileName.Content = String.Empty;
            Microsoft.Win32.OpenFileDialog dlg = new Microsoft.Win32.OpenFileDialog();
            dlg.DefaultExt = ".thv";
            dlg.Filter = "THV files (.thv)|*.thv";
            bool? result = dlg.ShowDialog();
            if (result == true)
            {
                lblFileName.Content = System.IO.Path.GetFileName(dlg.FileName);
                parseFile(dlg.FileName);
                drawThermalMap(imageScale);
            }
        }

        private void parseFile(String fileName)
        {
            thermalMap = new ThermalMap(fileName);
        }

        private void drawThermalMap(float scale)
        {
            float min = 400;
            float max = -100;
            mapWidth = thermalMap.Width;
            mapHeight = thermalMap.Height;
            int imageWidth = (int)Math.Round(mapWidth * scale);
            int imageHeight = (int)Math.Round(mapHeight * scale);
            Bitmap bitmap = new Bitmap(mapWidth, mapHeight, PixelFormat.Format24bppRgb);
            int i = 0;
            
            if (thermalMap.ThermMap.Count > 0)
            {
                foreach (var temp in thermalMap.ThermMap)
                {
                    if (temp < min)
                    {
                        min = temp;
                    }
                    if (temp > max)
                    {
                        max = temp;
                    }
                }
                i = 0;
                for (int y = 0; y < mapHeight; y++)
                {
                    for (int x = 0; x < mapWidth; x++)
                    {
                        bitmap.SetPixel(x, mapHeight - y - 1, calculateColor(ALTERNATE_COLOR_SCHEME, thermalMap.ThermMap[i], min, max));
                        i++;
                    }
                }
            }
            IntPtr hBitmap = bitmap.GetHbitmap();
            System.Windows.Controls.Image img = new System.Windows.Controls.Image();
            img.Source = System.Windows.Interop.Imaging.CreateBitmapSourceFromHBitmap(hBitmap, IntPtr.Zero, Int32Rect.Empty, BitmapSizeOptions.FromWidthAndHeight(imageWidth, imageHeight));
            canvas.Children.Clear();
            RemoveLogicalChild(img);
            canvas.Children.Add(img);
            img.MouseLeftButtonDown += img_MouseLeftButtonDown;
            img.MouseMove += img_MouseMove;
        }

        private static Color rgb565ToColor(UInt16 rgb)
        {
            var r = ((byte) (rgb >> 11)) << 3;
            var g = ((byte) ((rgb >> 5) & 0x003F)) << 2;
            var b = ((byte) (rgb & 0x001F)) << 3;
            return Color.FromArgb(r, g, b);
        }

        private byte calculateRGB(byte rgb1, byte rgb2, float t1, float step, float t)
        {
            return (byte)(rgb1 + (((t - t1) / step) * (rgb2 - rgb1)));
        }

        private Color calculateColor(Color[] colorScheme, float temperature, float min, float max)
        {
            Color val;
            if ((float.IsNaN(temperature)) || (temperature < min))
            {
                temperature = 0;
            }
            float step = (max - min) / 10.0f;
            if (temperature < min)
            {
                val = colorScheme[0];
            }
            else if (temperature >= max)
            {
                val = colorScheme[colorScheme.Count() - 1];
            }
            else
            {
                int step1 = (int)((temperature - min) / step);
                int step2 = step1 + 1;
                Color col1 = colorScheme[step1];
                Color col2 = colorScheme[step2];
                byte red = calculateRGB(col1.R, col2.R, (min + step1 * step), step, temperature);
                byte green = calculateRGB(col1.G, col2.G, (min + step1 * step), step, temperature);
                byte blue = calculateRGB(col1.B, col2.B, (min + step1 * step), step, temperature);
                val = Color.FromArgb(red, green, blue);
            }
            return val;
        }

        private void img_MouseMove(object sender, System.Windows.Input.MouseEventArgs e)
        {
            Point pos = e.GetPosition((IInputElement)sender);
            int x = (int)(pos.X / imageScale);
            int y = (int)(pos.Y / imageScale);
            int mapIdx = ((mapHeight - y) * mapWidth + x);
            if (mapIdx >= mapHeight * mapWidth)
            {
                mapIdx = mapHeight * mapWidth - 1;
            }
            lblTemperature.Content = String.Format("Temp:{0} C - Index: {1}", thermalMap.ThermMap[mapIdx], mapIdx);
        }

        private void img_MouseLeftButtonDown(object sender, System.Windows.Input.MouseButtonEventArgs e)
        {
            Point pos = e.GetPosition((IInputElement)sender);
            int x = (int) (pos.X / imageScale);
            int y = (int) (pos.Y / imageScale);
            int mapIdx = ((mapHeight - y) * mapWidth + x);
            if (mapIdx >= mapHeight * mapWidth)
            {
                mapIdx = mapHeight * mapWidth - 1;
            }
            lblTemperature.Content = String.Format("Temp:{0} C", thermalMap.ThermMap[mapIdx]);
        }

        private void canvas_MouseWheel(object sender, System.Windows.Input.MouseWheelEventArgs e)
        {
            if (e.Delta > 0)
            {
                imageScale += 0.1f;
                if (imageScale > 3)
                {
                    imageScale = 3;
                }
            }
            else
            {
                imageScale -= 0.1f;
                if (imageScale < 1)
                {
                    imageScale = 1.0f;
                }
            }
            canvas.Width *= imageScale; 
            canvas.Height *= imageScale; 
            drawThermalMap(imageScale);
        }
    }

}
