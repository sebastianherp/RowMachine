using System;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace RowMachineTacho
{
    /// <summary>
    /// Interaktionslogik für MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        bool listening = false;
        short port = 8888;
        const short broadcastPort = 12345;
        double speed = 0;
        double acceleration = 0;
        double anglePaddle = 0;
        double distance = 0;
        double targetDistance = 1000;
        const int circumfence = 1445; // in mm (23 cm radius)


        double speedMax = Double.MinValue;
        double speedMin = Double.MaxValue;

        UdpClient server;
        UdpClient broadcastListener = new UdpClient(new IPEndPoint(IPAddress.Any, broadcastPort));
        LinkedList<Tacho> speedList = new LinkedList<Tacho>();

        System.Windows.Threading.DispatcherTimer dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
        DateTime lastPacket = new DateTime();
        bool graphNeedsUpdate = false;

        class Tacho
        {
            public UInt16 tachoDiff;
            public UInt16 localDiff;
            public UInt16 counter;
            public double speed;

            public Tacho(UInt16 _tachoDiff, UInt16 _localDiff, UInt16 _counter)
            {
                tachoDiff = _tachoDiff;
                localDiff = _localDiff;
                counter = _counter;
                speed = 0;
                if (tachoDiff < 5000)
                    speed = (circumfence / 1000.0 / 1000.0) / (tachoDiff / 1000.0 / 3600.0);
            }

            public override string ToString()
            {
                return tachoDiff + ";" + localDiff + ";" + counter + ";" + speed.ToString("F2");
            }
        }

        public MainWindow()
        {
            InitializeComponent();


            startBroadcastListener();
            

            connect(port);

            Loaded += delegate
            {
                setupGame();
                setupGraph();
            };

            SizeChanged += delegate
            {
                setupGame();
            };


            UpdateUI();

            dispatcherTimer.Tick += dispatcherTimer_Tick;
            dispatcherTimer.Interval = new TimeSpan(0, 0, 0, 0, 50);
            dispatcherTimer.Start();
        }

        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            UpdateUI();
        }

        private void UpdateUI()
        {
            tbListening.Text = listening ? ("Listening on Port " + port) : "Stopped";

            distance = speedList.Count * circumfence / 1000.0;
            if (speedList.Last != null && speedList.Last.Previous != null)
            {
                Tacho last = speedList.Last.Value;
                Tacho nextToLast = speedList.Last.Previous.Value;

                if (graphNeedsUpdate)
                {
                    acceleration = (last.speed - nextToLast.speed) / last.tachoDiff * 1000 / 3.6;
                    speed = last.speed * 0.7 + speed * 0.3;
                    updateGraph(graphTachoSpeed, speed, 0, 50);
                    updateGraph(graphAcceleration, acceleration, 0, 50);
                    updateGraph(graphAngle, anglePaddle, -45, 45);

                    graphNeedsUpdate = false;
                }

                TimeSpan diff = DateTime.Now - lastPacket;
                if (diff.TotalMilliseconds > 5000 && speed != 0)
                {
                    speed = 0;
                    acceleration = 0;
                    updateGraph(graphTachoSpeed, speed, 0, 50);
                    updateGraph(graphAcceleration, acceleration, 0, 50);
                    updateGraph(graphAngle, anglePaddle, -45, 45);
                }

                



            }


            tbSpeed.Text = speed.ToString("F2") + " km/h";
            tbDistance.Text = distance.ToString("F0") + " m";

            updateGame();
        }

        private void stopBroadcastListener()
        {
            if (broadcastListener != null)
            {
                broadcastListener.Close();
                broadcastListener = null;
            }

        }

        private void startBroadcastListener()
        {
            stopBroadcastListener();

            broadcastListener = new UdpClient(new IPEndPoint(IPAddress.Any, broadcastPort));

            try
            {
                broadcastListener.BeginReceive(new AsyncCallback(recvBroadcast), null);
            }
            catch (Exception ex)
            {
                Console.WriteLine("Exception: " + ex.Message);
            }

        }

        private void connect(int _port) {

            if (server != null)
            {
                server.Close();
                server = null;
                listening = false;
                UpdateUI();
                Thread.Sleep(500);
            }

            IPEndPoint ipep = new IPEndPoint(IPAddress.Any, port);
            server = new UdpClient(ipep);

            try
            {
                server.BeginReceive(new AsyncCallback(recv), null);
                listening = true;
            }
            catch (Exception ex)
            {
                Console.WriteLine("Exception: " + ex.Message);
            }
        }

        private void reset()
        {
            speedList.Clear();
            speedMax = Double.MinValue;
            speedMin = Double.MaxValue;

            anglePaddle = 0;
            speed = 0;
            acceleration = 0;
        }

        private void btnListen_Click(object sender, RoutedEventArgs e)
        {
            if (!Int16.TryParse(tbPort.Text, out port))
                tbPort.Text = port.ToString();

            reset();
            connect(port);

        }

        private void recvBroadcast(IAsyncResult res)
        {
            IPEndPoint RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 8000);
            try
            {
                byte[] received = broadcastListener.EndReceive(res, ref RemoteIpEndPoint);

                if (received[0] == 123)
                {
                    if (server != null)
                    {
                        Console.WriteLine("Sending broadcast reply to " + RemoteIpEndPoint.Address.ToString() + ":" + RemoteIpEndPoint.Port);

                        server.Send(received, 1, RemoteIpEndPoint);
                    }
                    
                    
                }
            }
            catch (Exception e)
            {
                Console.WriteLine("Exception: " + e.Message);
            }

            if (broadcastListener != null)
                broadcastListener.BeginReceive(new AsyncCallback(recvBroadcast), null);
        }

        private void recv(IAsyncResult res)
        {
            IPEndPoint RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 8000);
            try
            {
                byte[] received = server.EndReceive(res, ref RemoteIpEndPoint);

                TimeSpan diff = DateTime.Now - lastPacket;
                lastPacket = DateTime.Now;

                UInt16 localDiff = 65000;
                if(diff.TotalMilliseconds < 65000)
                    localDiff = (UInt16)diff.TotalMilliseconds;

                //Process codes
                UInt16 value = BitConverter.ToUInt16(received, 0);
                UInt16 counter = BitConverter.ToUInt16(received, 2);
                speedList.AddLast(new Tacho(value, localDiff, counter));
                graphNeedsUpdate = true;
            }
            catch (Exception e)
            {
                Console.WriteLine("Exception: " + e.Message);
            }
            
            if(server != null)
                server.BeginReceive(new AsyncCallback(recv), null);
        }



        double xmin, xmax, ymin, ymax;
        const double stepX = 2;
        const double stepY = 10;

        private void setupGraph()
        {
            const double margin = 10;
            xmin = margin;
            xmax = canGraph.Width - margin;
            ymin = margin;
            ymax = canGraph.Height - margin;


            // Make the X axis.
            GeometryGroup xaxis_geom = new GeometryGroup();
            xaxis_geom.Children.Add(new LineGeometry(
                new Point(0, ymax), new Point(canGraph.Width, ymax)));
            for (double x = xmin + stepX;
                x <= canGraph.Width - stepX; x += stepX)
            {
                xaxis_geom.Children.Add(new LineGeometry(
                    new Point(x, ymax - margin / 2),
                    new Point(x, ymax + margin / 2)));
            }

            Path xaxis_path = new Path();
            xaxis_path.StrokeThickness = 1;
            xaxis_path.Stroke = Brushes.Black;
            xaxis_path.Data = xaxis_geom;

            canGraph.Children.Add(xaxis_path);

            // Make the Y ayis.
            GeometryGroup yaxis_geom = new GeometryGroup();
            yaxis_geom.Children.Add(new LineGeometry(
                new Point(xmin, 0), new Point(xmin, canGraph.Height)));
            for (double y = stepY; y <= canGraph.Height - stepY; y += stepY)
            {
                yaxis_geom.Children.Add(new LineGeometry(
                    new Point(xmin - margin / 2, y),
                    new Point(xmin + margin / 2, y)));
            }

            Path yaxis_path = new Path();
            yaxis_path.StrokeThickness = 1;
            yaxis_path.Stroke = Brushes.Black;
            yaxis_path.Data = yaxis_geom;

            canGraph.Children.Add(yaxis_path);


            graphTachoSpeed = setupGraph(Brushes.Red);
            graphAcceleration = setupGraph(Brushes.Green);
            graphAngle = setupGraph(Brushes.Blue);

            canGraph.Children.Add(graphTachoSpeed);
            canGraph.Children.Add(graphAcceleration);
            canGraph.Children.Add(graphAngle);

        }

        private Polyline graphTachoSpeed, graphAcceleration, graphAngle;

        private Polyline setupGraph(Brush b)
        {
            Polyline tempGraph = new Polyline();
            tempGraph.StrokeThickness = 1;
            tempGraph.Stroke = b;
            return tempGraph;
        }

        private double doubleMap(double x, double in_min, double in_max, double out_min, double out_max)
        {
            double temp = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            temp = (int)(4 * temp + .5);
            return (double)temp / 4;
        }

        private void updateGraph(Polyline graph, double value, double valueMin, double valueMax)
        {
            if (graph == null)
                return;

            int yCoordinate = (int)doubleMap(value, valueMin, valueMax, ymax, ymin);

            //(ymax - (ymax - ymin) * value / 1000.0);

            if (graph.Points.Count == 0)
            {
                // first element
                graph.Points.Add(new Point(xmin, yCoordinate));
            }
            else
            {
                double lastx = graph.Points[graph.Points.Count - 1].X;
                double nextx = lastx + stepX;
                if (nextx > xmax)
                {
                    //graphWatt.Points.RemoveAt(0);
                    for (int i = 0; i < graph.Points.Count - 1; i++)
                    {
                        graph.Points[i] = new Point(graph.Points[i].X, graph.Points[i + 1].Y);
                    }
                    graph.Points[graph.Points.Count - 1] = new Point(graph.Points[graph.Points.Count - 1].X, yCoordinate);
                }
                else
                {
                    graph.Points.Add(new Point(nextx, yCoordinate));
                }

            }
        }


        private Line gameWater;
        private Polyline gamePaddle;
        private Polygon gameBoat;
        private Polyline gameStickmanBody;
        private Ellipse gameStickmanHead;
        private int gameMargin = 10;
        private int gameMarginBottom = 70;

        private double getBoatPosition()
        {
            return gameMargin + (canGame.ActualWidth - gameMargin * 2 - gameBoatLength) * distance / targetDistance;
        }

        private void setupGame()
        {
            double width = canGame.ActualWidth;
            double height = canGame.ActualHeight;

            canGame.Children.Clear();


            gameWater = new Line();
            gameWater.Stroke = Brushes.Blue;
            gameWater.X1 = gameMargin;
            gameWater.X2 = width - gameMargin;
            gameWater.Y1 = height - gameMarginBottom;
            gameWater.Y2 = height - gameMarginBottom;
            gameWater.StrokeThickness = 10;


            gameBoat = new Polygon();
            gameBoat.Stroke = Brushes.Brown;
            gameBoat.StrokeThickness = 6;

            gamePaddle = new Polyline();
            gamePaddle.Stroke = Brushes.SaddleBrown;
            gamePaddle.StrokeThickness = 4;

            gameStickmanBody = new Polyline();
            gameStickmanBody.Stroke = Brushes.Black;
            gameStickmanBody.StrokeThickness = 2;

            gameStickmanHead = new Ellipse();
            gameStickmanHead.Stroke = Brushes.Black;
            gameStickmanHead.StrokeThickness = 2;


            drawBoat(width, height);



            canGame.Children.Add(gameBoat);
            canGame.Children.Add(gameWater);
            canGame.Children.Add(gameStickmanBody);
            canGame.Children.Add(gameStickmanHead);
            canGame.Children.Add(gamePaddle);
        }

        const double gameBoatLength = 110, gameBoatHeight = 20, gameBoatFront = 25, gameBoatBack = 5;


        private void drawBoat(double width, double height)
        {
            double boatX = getBoatPosition();
            double boatY = height - gameMarginBottom - 6;

            LinkedListNode<Tacho> last = speedList.Last;
            speedMax = Double.MinValue;
            speedMin = Double.MaxValue;

            for (int i = 0; i < 25; i++)
            {
                if (last == null)
                    break;
                speedMax = Math.Max(last.Value.speed, speedMax);
                speedMin = Math.Min(last.Value.speed, speedMin);
                last = last.Previous;
                    
            }

            anglePaddle = Math.Min(75, Math.Max(-45, doubleMap(speed, speedMin, speedMax, -45, 75)));

            if (gameBoat != null)
            {
                gameBoat.Points.Clear();
                gameBoat.Points.Add(new Point(boatX, boatY - gameBoatHeight));
                gameBoat.Points.Add(new Point(boatX + gameBoatBack, boatY));
                gameBoat.Points.Add(new Point(boatX + gameBoatLength - gameBoatFront, boatY));
                gameBoat.Points.Add(new Point(boatX + gameBoatLength, boatY - gameBoatHeight));
            }

            if (gamePaddle != null)
            {
                gamePaddle.Points.Clear();
                gamePaddle.Points.Add(new Point(boatX + gameBoatLength / 2, boatY - gameBoatHeight * 2));
                gamePaddle.Points.Add(new Point(boatX + gameBoatLength / 2, boatY + gameBoatHeight * 2));
                gamePaddle.RenderTransform = new RotateTransform(anglePaddle, boatX + gameBoatLength / 2, boatY - gameBoatHeight);

                double bodyXOffset = doubleMap(anglePaddle, -45, 75, 0, gameBoatLength / 4);

                gameStickmanHead.Width = gameBoatHeight;
                gameStickmanHead.Height = gameBoatHeight;
                Canvas.SetLeft(gameStickmanHead, boatX + gameBoatLength / 2 + bodyXOffset);
                Canvas.SetTop(gameStickmanHead, boatY - gameBoatHeight * 3);

                gameStickmanBody.Points.Clear();
                gameStickmanBody.Points.Add(new Point(boatX + gameBoatLength / 2 + bodyXOffset + gameStickmanHead.Width / 2, boatY - gameBoatHeight));
                gameStickmanBody.Points.Add(new Point(boatX + gameBoatLength / 2 + bodyXOffset + gameStickmanHead.Width / 2, boatY - gameBoatHeight * 3 + gameStickmanHead.Height));
                gameStickmanBody.Points.Add(gamePaddle.RenderTransform.Transform(gamePaddle.Points[0]));
            }

        }

        private void updateGame()
        {
            double width = canGame.ActualWidth;
            double height = canGame.ActualHeight;

            drawBoat(width, height);

        }

        private void btnDebug_Click(object sender, RoutedEventArgs e)
        {
            TimeSpan diff = DateTime.Now - lastPacket;
            lastPacket = DateTime.Now;

            UInt16 localDiff = 65000;
            if (diff.TotalMilliseconds < 65000)
                localDiff = (UInt16)diff.TotalMilliseconds;

            speedList.AddLast(new Tacho(localDiff, localDiff, 0));
            graphNeedsUpdate = true;
        }

    }
}
