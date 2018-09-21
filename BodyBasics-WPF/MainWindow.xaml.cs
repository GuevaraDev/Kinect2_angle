//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.IO.Ports;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        SerialPort port;
        String temp_1, temp_2;

        int a1_sig;
        int a2_sig;
        double hand_x, hand_y, hand_z, elbow_x, elbow_y, elbow_z, r_shoulder_x, r_shoulder_y, r_shoulder_z, l_shoulder_x, l_shoulder_y, l_shoulder_z;
        double[] v1 = new double[3];
        double[] v2 = new double[3];
        double[] v3 = new double[3];
        double angle_1, angle_2, angle1_3D, angle2_3D;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private void connectToArduino()
        {
            string selectedPort = "COM11";
            port = new SerialPort(selectedPort, 115200, Parity.None, 8, StopBits.One);
            port.Open();
            port.Write("#STAR\n");
        }

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            int c = 0;
            if (c == 0)
            {
                connectToArduino();
                c = 10;
            }

            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));

            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ShoulderRight));
            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));
            /*
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));
            
            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
            */
            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[2];

                        if (body.IsTracked)
                        {

                            Joint l_shoulder = body.Joints[JointType.ShoulderLeft];
                            Joint r_shoulder = body.Joints[JointType.ShoulderRight];
                            Joint elbow = body.Joints[JointType.ElbowRight];
                            Joint hand = body.Joints[JointType.WristRight];

                            hand_x = hand.Position.X;
                            hand_y = hand.Position.Y;
                            hand_z = hand.Position.Z;

                            elbow_x = elbow.Position.X;
                            elbow_y = elbow.Position.Y;
                            elbow_z = elbow.Position.Z;

                            r_shoulder_x = r_shoulder.Position.X;
                            r_shoulder_y = r_shoulder.Position.Y;
                            r_shoulder_z = r_shoulder.Position.Z;

                            l_shoulder_x = l_shoulder.Position.X;
                            l_shoulder_y = l_shoulder.Position.Y;
                            l_shoulder_z = l_shoulder.Position.Z;

                            //Vectores
                            v1[0] = (r_shoulder_x - l_shoulder_x);
                            v1[1] = (r_shoulder_y - l_shoulder_y);
                            v1[2] = (r_shoulder_z - l_shoulder_z);

                            v2[0] = (elbow_x - r_shoulder_x);
                            v2[1] = (elbow_y - r_shoulder_y);
                            v2[2] = (elbow_z - r_shoulder_z);

                            v3[0] = (hand_x - elbow_x);
                            v3[1] = (hand_y - elbow_y);
                            v3[2] = (hand_z - elbow_z);

                            Vector vector1 = new Vector(v1[0], v1[1]);
                            Vector vector2 = new Vector(v2[0], v2[1]);
                            Vector vector3 = new Vector(v3[0], v3[1]);
                            //Codo
                            angle_1 = Vector.AngleBetween(vector1, vector2);
                            //Mano
                            angle_2 = Vector.AngleBetween(vector2, vector3);

                            if (angle_1 > 0)
                            {
                                a1_sig = 1;
                            }
                            else
                            {
                                a1_sig = 0;
                            }

                            if (angle_2 > 0)
                            {
                                a2_sig = 1;
                            }
                            else
                            {
                                a2_sig = 0;
                            }


                            if (angle_1 <= -90)
                            {
                                angle_1 = -90;
                            } else if (angle_1 >= -91 && angle_1 < -83)
                            {
                                angle_1 = -90;
                            }
                            else if (angle_1 >= -83 && angle_1 < -76)
                            {
                                angle_1 = -76;
                            }
                            else if (angle_1 >= -75 && angle_1 < -70)
                            {
                                angle_1 = -75;
                            }
                            else if (angle_1 >= -70 && angle_1 < -65)
                            {
                                angle_1 = -70;
                            }
                            else if (angle_1 >= -65 && angle_1 < -60)
                            {
                                angle_1 = -65;
                            }
                            else if (angle_1 >= -60 && angle_1 < -55)
                            {
                                angle_1 = -60;
                            }
                            else if (angle_1 >= -55 && angle_1 < -50)
                            {
                                angle_1 = -55;
                            }
                            else if (angle_1 >= -50 && angle_1 < -45)
                            {
                                angle_1 = -50;
                            }
                            else if (angle_1 >= -45 && angle_1 < -40)
                            {
                                angle_1 = -45;
                            }
                            else if (angle_1 >= -40 && angle_1 < -35)
                            {
                                angle_1 = -40;
                            }
                            else if (angle_1 >= -35 && angle_1 < -30)
                            {
                                angle_1 = -35;
                            }
                            else if (angle_1 >= -30 && angle_1 < -25)
                            {
                                angle_1 = -30;
                            }
                            else if (angle_1 >= -25 && angle_1 < -20)
                            {
                                angle_1 = -25;
                            }
                            else if (angle_1 >= -20 && angle_1 < -15)
                            {
                                angle_1 = -20;
                            }
                            else if (angle_1 >= -15 && angle_1 < -10)
                            {
                                angle_1 = -15;
                            }
                            else if (angle_1 >= -10 && angle_1 < -5)
                            {
                                angle_1 = -10;
                            }
                            else if (angle_1 >= -5 && angle_1 < 0)
                            {
                                angle_1 = -5;
                            }
                            else if (angle_1 >= 0 && angle_1 < 5)
                            {
                                angle_1 = 0;
                            }
                            else if (angle_1 >= 5 && angle_1 < 10)
                            {
                                angle_1 = 5;
                            }
                            else if (angle_1 >= 10 && angle_1 < 15)
                            {
                                angle_1 = 15;
                            }
                            else if (angle_1 >= 15 && angle_1 < 20)
                            {
                                angle_1 = 20;
                            }
                            else if (angle_1 >= 20 && angle_1 < 25)
                            {
                                angle_1 = 25;
                            }
                            else if (angle_1 >= 25 && angle_1 < 30)
                            {
                                angle_1 = 30;
                            }
                            else if (angle_1 >= 30 && angle_1 < 35)
                            {
                                angle_1 = 35;
                            }
                            else if (angle_1 >= 35 && angle_1 < 40)
                            {
                                angle_1 = 40;
                            }
                            else if (angle_1 >= 45 && angle_1 < 50)
                            {
                                angle_1 = 45;
                            }
                            else if (angle_1 >= 50 && angle_1 < 55)
                            {
                                angle_1 = 50;
                            }
                            else if (angle_1 >= 55 && angle_1 < 60)
                            {
                                angle_1 = 55;
                            }
                            else if (angle_1 >= 60 && angle_1 < 65)
                            {
                                angle_1 = 60;
                            }
                            else if (angle_1 >= 65 && angle_1 < 70)
                            {
                                angle_1 = 65;
                            }
                            else if (angle_1 >= 70 && angle_1 < 75)
                            {
                                angle_1 = 70;
                            }
                            else if (angle_1 >= 75 && angle_1 < 80)
                            {
                                angle_1 = 75;
                            }
                            else if (angle_1 >= 80 && angle_1 < 85)
                            {
                                angle_1 = 80;
                            }
                            else if (angle_1 >= 85 && angle_1 < 90)
                            {
                                angle_1 = 85;
                            }
                            else if (angle_1 >= 90)
                            {
                                angle_1 = 90;
                            }
                            
                            //Fuzzy Angle Mano
                            if (angle_2 <= 0)
                            {
                                angle_2 = 0;
                            }
                            else if (angle_2 >= 9 && angle_2 < 18)
                            {
                                angle_2 = 9;
                            }
                            else if (angle_2 >= 18 && angle_2 < 27)
                            {
                                angle_2 = 18;
                            }
                            else if (angle_2 >= 27 && angle_2 < 36)
                            {
                                angle_2 = 27;
                            }
                            else if (angle_2 >= 36 && angle_2 < 45)
                            {
                                angle_2 = 36;
                            }
                            else if (angle_2 >= 45 && angle_2 < 54)
                            {
                                angle_2 = 45;
                            }
                            else if (angle_2 >= 54 && angle_2 < 63)
                            {
                                angle_2 = 54;
                            }
                            else if (angle_2 >= 63 && angle_2 < 72)
                            {
                                angle_2 = 63;
                            }
                            else if (angle_2 >= 72 && angle_2 < 81)
                            {
                                angle_2 = 72;
                            }
                            else if (angle_2 >= 81 && angle_2 < 90)
                            {
                                angle_2 = 81;
                            }
                            else if (angle_2 >= 90 && angle_2 < 99)
                            {
                                angle_2 = 90;
                            }
                            else if (angle_2 >= 99 && angle_2 < 108)
                            {
                                angle_2 = 99;
                            }
                            else if (angle_2 >= 108 && angle_2 < 117)
                            {
                                angle_2 = 108;
                            }
                            else if (angle_2 >= 117 && angle_2 < 126)
                            {
                                angle_2 = 117;
                            }
                            else if (angle_2 >= 126)
                            {
                                angle_2 = 126;
                            }


                            //Codo
                            temp_1 = (Convert.ToInt32(Math.Abs(angle_1))).ToString("D3");
                            //Mano
                            temp_2 = (Convert.ToInt32(Math.Abs(angle_2))).ToString("D3");

                            

                            //12345  6 7 8  9  10 11 12 13 14 15 16 17
                            //#TEXT  0 1 4  A  0  3  6  B  0  C  1  D#
                            

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:


                    port.Write("#TEXT" + temp_1 + "A" + temp_2 + "B" + a1_sig + "C" + a2_sig + "D" + "#\n");
                    //Console.WriteLine("#TEXT" + temp_1 + "A" + temp_2 + "B" + a1_sig + "C" + a2_sig + "D" + "#\n");
                    this.StatusText = String.Format("Angulo Codo: {0:F2}\nAngulo Mano: {1:F2}", angle_1, angle_2);
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:

                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
