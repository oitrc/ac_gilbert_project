using System.Linq;
using System.Windows;
using Microsoft.Research.Kinect.Nui;

namespace HandRaise
{
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
            m_time_ms = new double[m_num_frames];
            m_hand_y = new double[m_num_frames];
            m_speed_mps = new double[m_num_frames - 1];
            this.reset();
        }

        public void process_frame(SkeletonFrame skeleton_frame)
        {
            /*------------------------------------------------------
            Local variables
            ------------------------------------------------------*/
            float head_y;     /* head y position (meters)     */
            float handright_y;/* right hand y pos (meters)    */
            double timestamp_ms;
            /* timestamp (milliseconds)     */
            SkeletonData skeleton;   /* skeleton data of tracked user*/

            if (skeleton_frame == null)
            {
                return;
            }

            /*------------------------------------------------------
            Select the closest skeleton to track
            ------------------------------------------------------*/
            skeleton = (from s in skeleton_frame.Skeletons where s.TrackingState == SkeletonTrackingState.Tracked select s).FirstOrDefault();

            if (skeleton == null)
            {
                return;
            }

            head_y = skeleton.Joints[JointID.Head].Position.Y;
            handright_y = skeleton.Joints[JointID.HandRight].Position.Y;
            timestamp_ms = skeleton_frame.TimeStamp;

            if (handright_y > head_y)
            {
                /*------------------------------------------------------
                Hand is raised
                ------------------------------------------------------*/
                if (this.in_progress())
                {
                    /*------------------------------------------------------
                    Collect hand position samples
                    ------------------------------------------------------*/
                    this.set(handright_y, timestamp_ms);
                }
                else
                {
                    /*------------------------------------------------------
                    Handraise is complete
                    ------------------------------------------------------*/
                    MessageBox.Show(string.Format("hand raise speed (meters per second): {0:0.000000}", this.get_speed_mps()));
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
            for (int i = 0; i < m_speed_mps.Length; i++)
            {
                m_speed_mps[i] = this.get_speed_mps(i);
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
        private double get_speed_mps(int idx1, int idx2)
        {
            return (m_hand_y[idx2] - m_hand_y[idx1]) / ((m_time_ms[idx2] - m_time_ms[idx1]) / 1000);
        }

        /*------------------------------------------------------
        Return the speed in meters per second between two 
        consecutive data points given the first index
        ------------------------------------------------------*/
        private double get_speed_mps(int idx)
        {
            return this.get_speed_mps(idx, idx + 1);
        }

        private bool in_progress()
        {
            return m_frame_idx < m_num_frames;
        }

        private void set(double hand_y, double timestamp_ms)
        {
            m_time_ms[m_frame_idx] = timestamp_ms;
            m_hand_y[m_frame_idx] = hand_y;
            m_frame_idx++;
        }

        private int m_num_frames;
        /* number of frames to capture
           for each hand raise event    */
        private int m_frame_idx;/* frame index for successive 
                                           captures                     */
        private double[] m_time_ms;  /* timestamp array
                                           (milliseconds)               */
        private double[] m_hand_y;   /* right hand y position array
                                           (meters)                     */
        private double[] m_speed_mps;/* speed array (meters/second)  */
    };
}