#!/usr/bin/env python
import numpy as np
import math
# import scipy.interpolate as si
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from mpl_toolkits import mplot3d
import pickle
from datetime import datetime
from pytz import timezone

# Import crazyflie model modules
from a_cf_controller_phys import AltitudeControllerPhys, XYControllerPhys, YawControllerPhys, XYControllerTrajPhys
import sys
sys.path.append("../model/")
from data_plotter import DataPlotter
import crazyflie_param as P

# Import ros specifc modules
import rospy
from geometry_msgs.msg import Twist, Vector3, TransformStamped, PoseStamped # twist used in cmd_vel
from vicon_bridge.srv import viconGrabPose

class CooperativeQuad:
    def __init__(self, cf_name, is_main=False):
        rospy.init_node('test', anonymous=True)
        self.cf_name = cf_name
        self.msg = Twist()
        self.hz = 30.0
        self.t_phys = 1/self.hz # TODO make P.t_phys import
        self.rate = rospy.Rate(self.hz)
        
        # Requried for landing script
        self.x_fin = 0.0 
        self.y_fin = 0.0
        self.z_fin = 0.0
        self.yaw_fin = 0.0

        # Required based on syntax of launch script
        if is_main:
            self.pub = rospy.Publisher(self.cf_name + "/cmd_vel", Twist, queue_size=10)
        else:
            self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Required for vicon listener
        self.pose = TransformStamped()
        self.pose.transform.rotation.w = 1.0

        # Requied for publishPostion function
        self.pubPos = rospy.Publisher('cf_pos', PoseStamped, queue_size=1)

    def callback(self, pose):
        self.pose = pose
    
    def listener(self):
        rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
        pose = self.pose
        rospy.spin()
    
    # def callbackRef(self, pose):
    #     self.pos_ref = pose

    # def listener_ref(self, ref_object):
    #     rospy.Subscriber("/vicon/" + ref_object + "/" + ref_object, TransformStamped, self.callbackRef)
    #     pose_ref

    def dummyForLoop(self):
        """
        REQUIRED TO OVERCOME INITIAL PUBLISHER BLOCK IMPLEMENTED BY USC
        """
        self.msg.linear = Vector3(0, 0, 0)
        self.msg.angular = Vector3(0, 0, 0)
        for _ in range(100):
            self.pub.publish(self.msg)
            self.rate.sleep()
    
    # def publishPosition(self):
    #     # Start the vicon listener
    #     rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
    #     pose = self.pose


    def hoverStiff(self, x_c, y_c, z_c, yaw_c, goal_r, is_break=True, \
        is_synchronized=False, global_sync_time=0.0, time_delay=None, var=None):
        """
        Hovers the drone to an accurate global setpoint
        Drone will stay at setpoint until other function is called
        Stiff refers to optimization for global positional accuracy

        Parameters
        ----------
        x_c, y_c, z_c, yaw_c = reference setpoints
        goal_r = bounding radius for when drone is "close enough" to commanded setpoint
        is_break = will the loop break upon finding the setpoint or hover
            indefinitely
        is_synchornized = incorporate a universal time delay such that drones
            all start trajectory tracking at the same time
        global_sync_time = length of time delay for synchronization efforts
        """
        print(self.cf_name + ' started hover controller')

        rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
        pose = self.pose

        # Initialize required hover controllers
        altitude_ctrl_phys = AltitudeControllerPhys()
        xy_ctrl_phys = XYControllerPhys()
        yaw_ctrl_phys = YawControllerPhys()

        # for is_synchronized overload
        tz = timezone('EST')
        now = datetime.now(tz)
        secs = str(now.second) + '.' + str(now.microsecond)
        local_now = now.hour * 3600 + now.minute * 60 + float(secs)
        local_in_loop = local_now
        
        # for time_delay overload
        local2 = 0.0
        
        while not rospy.is_shutdown():
            pose_prev = pose
            pose = self.pose
            print(pose.transform.translation)
            quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
            x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
            if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
                pose = pose_prev

            # Obtain yaw angle from quaternion
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            if var=='z':
                self.msg.linear.z = altitude_ctrl_phys.update(z_c, z, is_pickling=True)
            else:
                self.msg.linear.z = altitude_ctrl_phys.update(z_c, z)
            
            if var == 'xy':
                self.msg.linear.x, self.msg.linear.y = xy_ctrl_phys.update(x_c, x, y_c, y, yaw, is_pickling=True)
            else:
                self.msg.linear.x, self.msg.linear.y = xy_ctrl_phys.update(x_c, x, y_c, y, yaw)
            
            if var=='yaw':
                self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw, is_pickling=True)
            else:
                self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)

            # Uncomment to set better trim angles
            # self.msg.linear.x = 0.0; self.msg.linear.y = 0.0 # set to tune trim values

            ### Goal behavior ###
            if is_break:
                if (x > (x_c - goal_r) and x < (x_c + goal_r)) and \
                    (y > (y_c - goal_r) and y < (y_c + goal_r)) and \
                    (z > (z_c - goal_r) and z < (z_c + goal_r)):
                    print(self.cf_name + ' found the hover setpoint!')
                    # Save out last known location
                    self.x_fin = x; self.y_fin = y; self.yaw_fin = yaw

                    break # include to move to other function
            
            # print("global_sync_time is: {}".format(global_sync_time))
            # print("local_in_loop is: {}".format(local_in_loop))
            if is_synchronized:
                local_in_loop += self.t_phys
                time_offset = 0.04
                print(local_in_loop)
                print(global_sync_time - time_offset)
                print(global_sync_time + time_offset)
                if (local_in_loop > (global_sync_time - time_offset) \
                    and local_in_loop < (global_sync_time + time_offset)):
                    print(self.cf_name + ' hit the time delay for synchronization!')
                    break
            
            if time_delay != None:
                local2 += self.t_phys
                time_offset = 0.4
                if (time_delay > (local2 - time_offset)) and \
                    (time_delay < (local2 + time_offset)):
                    print(self.cf_name + ' hit the time delay!')
                    break
            
            self.pub.publish(self.msg)
            self.rate.sleep()
        
        if var=='z':
            print('altitude controller is pickling')
            altitude_ctrl_phys.pickleData()
            print('altitude controller pickling completed!')
        if var=='yaw':
            print('yaw controller is pickling')
            yaw_ctrl_phys.pickleData()
            print('yaw controller pickling completed!')
        if var=='xy':
            print('xy controller is pickling')
            xy_ctrl_phys.pickleData()
            print('xy controller pickling completed!')   

    def trajTrackingStandingWave(self, traj, z_c, y_c=0.0):
        """
        Runs a 2D trajectory tracking algorithm in the XY plane

        Parameters
        ----------
        traj = trajectory that increments at each loop iteration
        z_c  = commanded height value
        y_c  = commanded y-offset value if traj only operates in x direction
        """
        print(self.cf_name + ' started trajectory tracking!')

        rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
        pose = self.pose

        # Initialize required controllers
        altitude_ctrl_phys = AltitudeControllerPhys()
        xy_traj_ctrl_phys = XYControllerTrajPhys()
        yaw_ctrl_phys = YawControllerPhys()
        
        y_c = y_c; v_c = 0.0; vd_c = 0.0 # keep y values equal to zero for now 
        yaw_c = 0.0

        # Will finish at end of trajectory matrix, 1 entry per loop interation
        for i in range(traj.shape[0] - 1):
            pose_prev = pose
            pose = self.pose
            quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
            x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
            if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
                pose = pose_prev
        
            # Obtain yaw angle from quaternion
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            # TODO: make flexible with y values
            r_t      = np.array([traj[i, 0], y_c]) # traj pos values
            r_t_vect = np.array([traj[i+1, 0], y_c]) - r_t # vector from current pos to next pos in traj
            rd_t     = np.array([traj[i, 1], v_c]) # traj vel values
            rdd_t    = np.array([traj[i, 2], vd_c])
            r        = np.array([x, y]) # actual drone pos

            self.msg.linear.z = altitude_ctrl_phys.update(z_c, z)
            self.msg.linear.x, self.msg.linear.y = xy_traj_ctrl_phys.update(r_t, rd_t, r_t_vect, r, yaw_c, rdd_t, True)
            self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)

            # print('theta (commands +x) is: ', self.msg.linear.x)
            # print('phi   (commands -y) is: ', self.msg.linear.y)

            # self.msg.linear.x = 0.0; self.msg.linear.y = 0.0 # set to tune trim values

            self.pub.publish(self.msg)
            self.rate.sleep()

        print(self.cf_name + ' completed trajectory tracking!')

        # Save out data through pickle
        xy_traj_ctrl_phys.pickleData()




    def trajTracking(self, traj, z_c):
        """
        Runs a 2D-3D trajectory tracking algorithm

        Parameters
        ----------
        traj = trajectory that increments at each loop iteration in form
        [ x, y, z, xd, yd, zd, xdd, ydd, zdd ]
        ref_object = where position readings are referenced, default is the world origin
        """
        print(self.cf_name + ' started trajectory tracking!')

        rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
        pose = self.pose

        rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)

        # Initialize required controllers
        altitude_ctrl_phys = AltitudeControllerPhys()
        xy_traj_ctrl_phys = XYControllerTrajPhys()
        yaw_ctrl_phys = YawControllerPhys()

        yaw_c = 0.0

        # Will finish at end of trajectory matrix, 1 entry per loop interation
        for i in range(traj.shape[0] - 1):
            pose_prev = pose
            pose = self.pose
            quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
            x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
            if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
                pose = pose_prev
        
            # Obtain yaw angle from quaternion
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            #cf1.hoverStiff(0.0, 0.0, 0.4, 0.0, 0.01) # False means don't break after reaching set point
            r_t      = np.array([traj[i, 0], traj[i, 3]]) # traj pos values
            r_t_vect = np.array([traj[i+1, 0], traj[i+1, 3]]) - r_t # vector from current pos to next pos in traj
            rd_t     = np.array([traj[i, 1], traj[i, 4]]) # traj vel values
            rdd_t    = np.array([traj[i, 2], traj[i, 5]])
            r        = np.array([x, y]) # actual drone pos

            self.msg.linear.z = altitude_ctrl_phys.update(z_c, z)
            self.msg.linear.x, self.msg.linear.y = xy_traj_ctrl_phys.update(r_t, rd_t, r_t_vect, r, yaw_c, rdd_t, is_pickling=True)
            self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)

            # print('theta (commands +x) is: ', self.msg.linear.x)
            # print('phi   (commands -y) is: ', self.msg.linear.y)

            self.pub.publish(self.msg)
            self.rate.sleep()

        print(self.cf_name + ' completed trajectory tracking!')

        # Save out data through pickle
        xy_traj_ctrl_phys.pickleData()

    def land(self):
        """
        Take last known x and y coordinates from hover script
        and slowly land the drone while maintaining xy control
        """
        print(self.cf_name + ' started landing!')

        rospy.Subscriber("/vicon/" + self.cf_name + "/" + self.cf_name, TransformStamped, self.callback)
        pose = self.pose

        # Initialize required hover controllers
        xy_ctrl_phys = XYControllerPhys(cap=5.0)
        yaw_ctrl_phys = YawControllerPhys()

        x_c = self.x_fin; y_c = self.y_fin; yaw_c = self.yaw_fin
        self.msg.linear.z = 40000.0
        
        while not rospy.is_shutdown():
            pose_prev = pose
            pose = self.pose
            quat = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
            x = pose.transform.translation.x; y = pose.transform.translation.y; z = pose.transform.translation.z
            if math.isnan(pose.transform.translation.x): # handle nans by setting to last known position
                pose = pose_prev

            # Obtain yaw angle from quaternion
            R = Rotation.from_quat(quat)
            x_global = R.apply([1, 0, 0]) # project to world x-axis
            yaw = np.arctan2(np.cross([1, 0, 0], x_global)[2], np.dot(x_global, [1, 0, 0]))

            self.msg.linear.x, self.msg.linear.y = xy_ctrl_phys.update(x_c, x, y_c, y, yaw)
            self.msg.angular.z = yaw_ctrl_phys.update(yaw_c, yaw)
                
            if self.msg.linear.z > 35000.0:
                self.msg.linear.z -= 150.0
            else:    
                print(self.cf_name + ' completed landing!')
                break

            self.pub.publish(self.msg)
            self.rate.sleep()

def main():
    """
    This runs only with a_hover_stiff.launch for crazyflie3
    """
    try:
        # Initialize drone control class with arg matching vicon object name
        cf1 = CooperativeQuad('crazyflie3', True)
        cf1.dummyForLoop()

        # # Hover at z=0.5, works tested 1/27/2020
        cf1.hoverStiff(0.3, 0.0, 0.4, 0.0, 0.01) # False means don't break after reaching set point
        
        # # z test
        # cf1.hoverStiff(0.0, 0.0, 1.4, 0.0, 0.01, is_break=False, time_delay=10.0, var='z') # x # y # yaw
        
        # # yaw test
        # cf1.hoverStiff(0.0, 0.0, 0.4, 1.0, 0.1, is_break=False, time_delay=10.0, var='yaw')

        # # x test
        # cf1.hoverStiff(1.0, 0.0, 0.4, 0.0, 0.01, is_break=False, time_delay=10.0, var='xy')

        # # y test
        # cf1.hoverStiff(0.0, 1.0, 0.4, 0.0, 0.01, is_break=False, time_delay=10.0, var='xy')
        
        cf1.land()

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()