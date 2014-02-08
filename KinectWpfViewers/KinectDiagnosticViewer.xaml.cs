﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using Microsoft.Research.Kinect.Nui;
using KinectNui = Microsoft.Research.Kinect.Nui;


public class hand_raise
{
    public hand_raise()
    {
        /*------------------------------------------------------
        Statically set the number of frames to use which really
        depends on the following factors:
         *  camera framerate (in frames per second)
         *  maximum hand raise speed
         *  hand raise length (between centers of head/hand)
        
        Notes: 
         *  The main purpose of taking multiple samples is to
            reduce error in the hand Y position by averaging the
            speed over the course of the hand raise.  The more
            samples the more accurate the speed measurement.
         *  The main problem with the approach of using a set 
            number of frames is if the hand raise is very fast
            we will average in the time it reaches the top and
            stops, so it may be necessary to profile the hand
            raise in order to only average the appropriate
            number of frames.
        
        Calculation for why 3 is a good number of frames for
        children.
        Assumptions:
         *  camera framerate of 30 fps
         *  maximum hand raise speed of 3 m/s
                -> 118 inches per second
         *  hand raise length is about 12 inches
        Calculation:
         *  118/30  -> 4 inches per frame at max speed
         *  12/4    -> 3 frames for full hand raise
        ------------------------------------------------------*/
        m_num_frames = 3;
        m_time_ms    = new double[ m_num_frames ];
        m_hand_y     = new double[ m_num_frames ];
        m_speed_mps  = new double[ m_num_frames - 1 ];
        this.reset();
    }

    public void process_frame( SkeletonFrame skeleton_frame )
    {
        float           head_y;
        float           handright_y;
        double          timestamp_ms;
        SkeletonData    skeleton;

        if( skeleton_frame == null )
        {
            return;
        }

        /*------------------------------------------------------
        Select the closest skeleton to track
        ------------------------------------------------------*/
        skeleton = ( from s in skeleton_frame.Skeletons where s.TrackingState == SkeletonTrackingState.Tracked select s ).FirstOrDefault();

        head_y       = skeleton.Joints[ JointID.Head ].Position.Y;
        handright_y  = skeleton.Joints[ JointID.HandRight ].Position.Y;
        timestamp_ms = skeleton_frame.TimeStamp;

        if( handright_y > head_y )
        {
            /*------------------------------------------------------
            Hand is raised
            ------------------------------------------------------*/
            if( this.in_progress() )
            {
                /*------------------------------------------------------
                Collect hand position samples
                ------------------------------------------------------*/
                this.set( handright_y, timestamp_ms );
            }
            else
            {
                /*------------------------------------------------------
                Handraise is complete
                ------------------------------------------------------*/
                MessageBox.Show( string.Format( "hand raise speed (meters per second): {0:0.000000}", this.get_speed_mps() ) );
                this.reset();
            }
        }
    }

    private void reset()
    {
        m_frame_idx = 0;
    }

    /*------------------------------------------------------
    Return the total average speed in meters per second
    ------------------------------------------------------*/
    private double get_speed_mps()
    {
        /*------------------------------------------------------
        Calculate speeds for each interval
        ------------------------------------------------------*/
        for( int i = 0; i < m_speed_mps.Length; i++ )
        {
            m_speed_mps[ i ] = this.get_speed_mps( i );
        }

        /*------------------------------------------------------
        Return the average speed of the intervals
        ------------------------------------------------------*/
        return m_speed_mps.Average();
    }

    /*------------------------------------------------------
    Return the speed in meters per second between two data
    points given their indices, where idx1 < idx2
    ------------------------------------------------------*/
    private double get_speed_mps( int idx1, int idx2 )
    {
        return ( m_hand_y[ idx2 ] - m_hand_y[ idx1 ] ) / ( ( m_time_ms[ idx2 ] - m_time_ms[ idx1 ] ) / 1000 );
    }

    /*------------------------------------------------------
    Return the speed in meters per second between two 
    consecutive data points given the first index
    ------------------------------------------------------*/
    private double get_speed_mps( int idx )
    {
        return this.get_speed_mps( idx, idx + 1 );
    }

    private bool in_progress()
    {
        return ( m_frame_idx < m_num_frames );
    }

    private void set( double hand_y, double timestamp_ms )
    {
        m_time_ms[ m_frame_idx ] = timestamp_ms;
        m_hand_y[ m_frame_idx ]  = hand_y;
        m_frame_idx++;
    }

    private int         m_num_frames;
    private int         m_frame_idx;
    private double[]    m_time_ms;
    private double[]    m_hand_y;
    private double[]    m_speed_mps;
};


namespace Microsoft.Samples.Kinect.WpfViewers
{
    /// <summary>
    /// Interaction logic for KinectDiagnosticViewer.xaml
    /// </summary>
    public partial class KinectDiagnosticViewer : UserControl
    {
        #region Public API
        public KinectDiagnosticViewer()
        {
            InitializeComponent();
        }

        public RuntimeOptions RuntimeOptions { get; private set; }

        public void ReInitRuntime()
        {
            // Will call Uninitialize followed by Initialize.
            this.Kinect = this.Kinect;
        }

        public KinectNui.Runtime Kinect
        {
            get { return _Kinect; }
            set
            {
                //Clean up existing runtime if we are being set to null, or a new Runtime.
                if (_Kinect != null)
                {
                    kinectColorViewer.Kinect = null;
                    kinectDepthViewer.Kinect = null;                    
                    _Kinect.SkeletonFrameReady -= new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
                    _Kinect.Uninitialize();
                }

                _Kinect = value;

                if (_Kinect != null)
                {
                    InitRuntime();

                    kinectColorViewer.Kinect = _Kinect;

                    kinectDepthViewer.RuntimeOptions = RuntimeOptions;
                    kinectDepthViewer.Kinect = _Kinect;

                    _Kinect.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);

                    UpdateUi();
                }
            }
        }

        //Status and InstanceIndex can change. Those properties in the Runtime should be made to support INotifyPropertyChange.
        public void UpdateUi()
        {
            kinectIndex.Text = _Kinect.InstanceIndex.ToString();
            kinectName.Text = _Kinect.InstanceName;
            kinectStatus.Text = _Kinect.Status.ToString();
        }

        hand_raise hr = new hand_raise();

        #endregion Public API

        #region Init
        private void InitRuntime()
        {
            //Some Runtimes' status will be NotPowered, or some other error state. Only want to Initialize the runtime, if it is connected.
            if (_Kinect.Status == KinectStatus.Connected)
            {
                bool skeletalViewerAvailable = IsSkeletalViewerAvailable;

                // NOTE:  Skeletal tracking only works on one Kinect per process right now.
                RuntimeOptions = skeletalViewerAvailable ?
                                     RuntimeOptions.UseDepthAndPlayerIndex | RuntimeOptions.UseSkeletalTracking | RuntimeOptions.UseColor
                                     : RuntimeOptions.UseDepth | RuntimeOptions.UseColor;
                _Kinect.Initialize(RuntimeOptions);
                skeletonPanel.Visibility = skeletalViewerAvailable ? System.Windows.Visibility.Visible : System.Windows.Visibility.Collapsed;
                if (RuntimeOptions.HasFlag(RuntimeOptions.UseSkeletalTracking))
                {
                    _Kinect.SkeletonEngine.TransformSmooth = true;
                }
            }
        }
        
        /// <summary>
        /// Skeletal tracking only works on one Kinect right now.  So return false if it is already in use.
        /// </summary>
        private bool IsSkeletalViewerAvailable
        {
            get { return KinectNui.Runtime.Kinects.All(k => k.SkeletonEngine == null); }
        }

        #endregion Init

        #region Skeleton Processing
        private void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame skeletonFrame = e.SkeletonFrame;
            
            //KinectSDK TODO: this shouldn't be needed, but if power is removed from the Kinect, you may still get an event here, but skeletonFrame will be null.
            if (skeletonFrame == null)
            {
                return;
            }

            int iSkeleton = 0;
            Brush[] brushes = new Brush[6];
            brushes[0] = new SolidColorBrush(Color.FromRgb(255, 0, 0));
            brushes[1] = new SolidColorBrush(Color.FromRgb(0, 255, 0));
            brushes[2] = new SolidColorBrush(Color.FromRgb(64, 255, 255));
            brushes[3] = new SolidColorBrush(Color.FromRgb(255, 255, 64));
            brushes[4] = new SolidColorBrush(Color.FromRgb(255, 64, 255));
            brushes[5] = new SolidColorBrush(Color.FromRgb(128, 128, 255));

            skeletonCanvas.Children.Clear();
            foreach (SkeletonData data in skeletonFrame.Skeletons)
            {
                if (SkeletonTrackingState.Tracked == data.TrackingState)
                {
                    // Draw bones
                    Brush brush = brushes[iSkeleton % brushes.Length];
                    skeletonCanvas.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.Spine, JointID.ShoulderCenter, JointID.Head));
                    skeletonCanvas.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderLeft, JointID.ElbowLeft, JointID.WristLeft, JointID.HandLeft));
                    skeletonCanvas.Children.Add(getBodySegment(data.Joints, brush, JointID.ShoulderCenter, JointID.ShoulderRight, JointID.ElbowRight, JointID.WristRight, JointID.HandRight));
                    skeletonCanvas.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipLeft, JointID.KneeLeft, JointID.AnkleLeft, JointID.FootLeft));
                    skeletonCanvas.Children.Add(getBodySegment(data.Joints, brush, JointID.HipCenter, JointID.HipRight, JointID.KneeRight, JointID.AnkleRight, JointID.FootRight));

                    // Draw joints
                    foreach (Joint joint in data.Joints)
                    {
                        Point jointPos = getDisplayPosition(joint);
                        Line jointLine = new Line();
                        jointLine.X1 = jointPos.X - 3;
                        jointLine.X2 = jointLine.X1 + 6;
                        jointLine.Y1 = jointLine.Y2 = jointPos.Y;
                        jointLine.Stroke = jointColors[joint.ID];
                        jointLine.StrokeThickness = 6;
                        skeletonCanvas.Children.Add(jointLine);
                    }
                }
                iSkeleton++;
            } // for each skeleton

            hr.process_frame( skeletonFrame );
        }

        private Polyline getBodySegment(Microsoft.Research.Kinect.Nui.JointsCollection joints, Brush brush, params JointID[] ids)
        {
            PointCollection points = new PointCollection(ids.Length);
            for (int i = 0; i < ids.Length; ++i)
            {
                points.Add(getDisplayPosition(joints[ids[i]]));
            }

            Polyline polyline = new Polyline();
            polyline.Points = points;
            polyline.Stroke = brush;
            polyline.StrokeThickness = 5;
            return polyline;
        }

        private Point getDisplayPosition(Joint joint)
        {
            float depthX, depthY;
            Kinect.SkeletonEngine.SkeletonToDepthImage(joint.Position, out depthX, out depthY);
            depthX = depthX * 320; //convert to 320, 240 space
            depthY = depthY * 240; //convert to 320, 240 space
            int colorX, colorY;
            ImageViewArea iv = new ImageViewArea();
            // only ImageResolution.Resolution640x480 is supported at this point
            Kinect.NuiCamera.GetColorPixelCoordinatesFromDepthPixel(ImageResolution.Resolution640x480, iv, (int)depthX, (int)depthY, (short)0, out colorX, out colorY);

            // map back to skeleton.Width & skeleton.Height
            return new Point((int)(skeletonCanvas.Width * colorX / 640.0), (int)(skeletonCanvas.Height * colorY / 480));
        }

        private static Dictionary<JointID, Brush> jointColors = new Dictionary<JointID, Brush>() { 
            {JointID.HipCenter, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.Spine, new SolidColorBrush(Color.FromRgb(169, 176, 155))},
            {JointID.ShoulderCenter, new SolidColorBrush(Color.FromRgb(168, 230, 29))},
            {JointID.Head, new SolidColorBrush(Color.FromRgb(200, 0,   0))},
            {JointID.ShoulderLeft, new SolidColorBrush(Color.FromRgb(79,  84,  33))},
            {JointID.ElbowLeft, new SolidColorBrush(Color.FromRgb(84,  33,  42))},
            {JointID.WristLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HandLeft, new SolidColorBrush(Color.FromRgb(215,  86, 0))},
            {JointID.ShoulderRight, new SolidColorBrush(Color.FromRgb(33,  79,  84))},
            {JointID.ElbowRight, new SolidColorBrush(Color.FromRgb(33,  33,  84))},
            {JointID.WristRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.HandRight, new SolidColorBrush(Color.FromRgb(37,   69, 243))},
            {JointID.HipLeft, new SolidColorBrush(Color.FromRgb(77,  109, 243))},
            {JointID.KneeLeft, new SolidColorBrush(Color.FromRgb(69,  33,  84))},
            {JointID.AnkleLeft, new SolidColorBrush(Color.FromRgb(229, 170, 122))},
            {JointID.FootLeft, new SolidColorBrush(Color.FromRgb(255, 126, 0))},
            {JointID.HipRight, new SolidColorBrush(Color.FromRgb(181, 165, 213))},
            {JointID.KneeRight, new SolidColorBrush(Color.FromRgb(71, 222,  76))},
            {JointID.AnkleRight, new SolidColorBrush(Color.FromRgb(245, 228, 156))},
            {JointID.FootRight, new SolidColorBrush(Color.FromRgb(77,  109, 243))}
        };
        #endregion Skeleton Processing

        #region Private State
        private KinectNui.Runtime _Kinect;
        #endregion Private State
    }
}
