#!/usr/bin/python3

## Get image data
## to the "imu_data" topic

import rospy
from std_msgs.msg import Int8
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Pose
from raceon.msg import TrackPosition

# Dependencies for estimation
import numpy as np
from scipy.signal import find_peaks, butter, filtfilt

class PosEstimator():
    
    def __init__(self):
        self.topic_name_camera_image = rospy.get_param("topic_name_camera_image", "camera/image")
        self.topic_name_pos_err = rospy.get_param("topic_name_position_error", "position/error")
        self.topic_name_pos_track = rospy.get_param("topic_name_position_track", "position/track")
        self.topic_name_pos_state = rospy.get_param("topic_name_position_state", "position/state")
        self.frame_name = rospy.get_param("frame_name", "camera")
        
        # Parameters for estimation
        self.scan_line_d = rospy.get_param("~scan_line_d", 170)
        self.peak_thres_d = rospy.get_param("~peak_threshold_d", 170)
        self.track_width = rospy.get_param("~track_width", 600)
        self.camera_center = rospy.get_param("~camera_center", 320)
        
        self.breaker = rospy.get_param("~breaker", 0);
        
        # line2
        self.scan_line_u = rospy.get_param("~scan_line_u", 150)
        self.peak_thres_u = rospy.get_param("~peak_threshold_u", 170)
        
        
        self.butter_b, self.butter_a = butter(3, 0.1)
        
        self.last_line_pos = self.camera_center
    
    def start(self):
        self.sub_camera = rospy.Subscriber(self.topic_name_camera_image, Image, self.image_callback)
            
        self.pub_pos_err = rospy.Publisher(self.topic_name_pos_err, Pose, queue_size=10)
        self.pub_pos_track = rospy.Publisher(self.topic_name_pos_track, TrackPosition, queue_size=10)
        self.pub_pos_state = rospy.Publisher(self.topic_name_pos_state, Int8, queue_size=10)
        rospy.spin()

    def image_callback(self, img_msg):
        width = img_msg.width
        height = img_msg.height
        
        np_arr = np.frombuffer(img_msg.data, dtype=np.uint8)
        img = np_arr.reshape(height, width)
        self.process_image(img)
    
    def process_image(self, img):
        rospy.loginfo("Image with shape {:s} received. (max, min)=({:d}, {:d})".format(str(img.shape), img.min(), img.max()))
        line_pos, state = self.pos_estimate(img)
        pos_err = self.camera_center - line_pos
        
        rospy.loginfo("Estimated line_pos = {:d}, state = {:d}".format(line_pos, state))
        
        pos_msg = Pose()
        pos_msg.position.x = pos_err
        self.pub_pos_err.publish(pos_msg)
        
        state_msg = Int8()
        state_msg.data = state
        self.pub_pos_state.publish(state_msg)
        
    def pos_estimate(self, I):
        # Select a horizontal line in the middle of the image
        L = I[self.scan_line_d, :]

        # Smooth the transitions so we can detect the peaks 
        Lf = filtfilt(self.butter_b, self.butter_a, L)

        # Find peaks which are higher than 0.5
        peaks_d, p_val1 = find_peaks(Lf, height=self.peak_thres_d)
        
        peaks_u, p_val2 = find_peaks(Lf, height=self.peak_thres_u)
        
        rospy.loginfo(peaks_d)

        state       = 0
        line_pos    = self.last_line_pos
        line_left_d   = None
        line_right_d  = None
        peaks_left_d  = peaks_d[peaks_d < self.last_line_pos]
        peaks_right_d = peaks_d[peaks_d > self.last_line_pos]
        
        line_left_u   = None
        line_right_u  = None
        peaks_left_u  = peaks_u[peaks_u < self.last_line_pos]
        peaks_right_u = peaks_u[peaks_u > self.last_line_pos]
        
        # Peaks on the left
        if peaks_left_d.size:
            line_left_d = peaks_left_d.max()
        if peaks_left_u.size:
            line_left_u = peaks_left_u.max()

        # Peaks on the right
        if peaks_right_d.size:
            line_right_d = peaks_right_d.min()
        if peaks_right_u.size:
            line_right_u = peaks_right_u.min()
        
        # Log track position
        track_msg = TrackPosition()
        track_msg.left = 0 if line_left_d == None else int(line_left_d)
        track_msg.right = 0 if line_right_d == None else int(line_right_d)
        self.pub_pos_track.publish(track_msg)

        # Evaluate the line position
        if line_left_d and line_right_d:
            line_pos    = (line_left_d + line_right_d ) // 2
            if line_left_u and line_right_u:
                self.breaker = 0
                state = 1
            else:
                if self.breaker < 5:
                    state = 2
                    self.breaker = self.breaker + 1
                else:
                    state = 3
                    
            
        elif line_left_d and not line_right_d:
            if self.breaker < 5:
                    state = 2
                    self.breaker = self.breaker + 1
            else:
                line_pos    = line_left_d + int(self.track_width / 2)
                state = 4
            
        elif not line_left_d and line_right_d:
            if self.breaker < 5:
                    state = 2
                    self.breaker = self.breaker + 1
            else:
                line_pos    = line_right_d - int(self.track_width / 2)
                state = 5
            
        else:
            rospy.loginfo("no line")
        
        self.last_line_pos = line_pos
        return line_pos, state

if __name__ == "__main__":
    rospy.init_node("pos_estimation")
    estimator = PosEstimator()
    try:
        estimator.start()
    except rospy.ROSInterruptException:
        pass
