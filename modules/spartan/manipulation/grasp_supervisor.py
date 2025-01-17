# system
import os
import numpy as np
import random
import copy
import time

# ROS
import rospy
import std_msgs.msg
import sensor_msgs.msg
import geometry_msgs.msg
import visualization_msgs.msg
import tf2_ros
import rosbag
import actionlib
from actionlib_msgs.msg import GoalStatus
import ros_numpy

# spartan ROS
import spartan_grasp_msgs.msg
import spartan_grasp_msgs.srv

import pdc_ros_msgs.msg

import fusion_server.msg
import fusion_server.srv

# spartan
import spartan.utils.utils as spartanUtils
import spartan.utils.ros_utils as rosUtils
import spartan.utils.director_utils as director_utils
import spartan.utils.control_utils as control_utils

from spartan.manipulation.schunk_driver import SchunkDriver
import fusion_server
from fusion_server.srv import *
import spartan.manipulation.gripper
from spartan.poser.poser_visualizer import PoserVisualizer
from spartan.manipulation.grasp_data import GraspData
from spartan.manipulation.object_manipulation import ObjectManipulation
from spartan.manipulation.category_manipulation_type import CategoryManipulationType
from spartan.utils.director_ros_visualizer import DirectorROSVisualizer


# director
from director import transformUtils
from director import visualization as vis
import director.objectmodel as om
import director.vtkNumpy as vnp
from director.debugVis import DebugData
import director.vtkAll as vtk
import director.segmentation as segmentation
import director.filterUtils as filterUtils

USING_DIRECTOR = True
if USING_DIRECTOR:
    from spartan.utils.taskrunner import TaskRunner


MUG_RACK_CONFIG_FILE = os.path.join(spartanUtils.getSpartanSourceDir(), "src/catkin_projects/station_config/RLG_iiwa_1/manipulation/mug_rack.yaml")

# IF true limits you to this speed
DEBUG_SPEED = 20 # degrees per second
USE_DEBUG_SPEED = False

MANIP_TYPE = CategoryManipulationType.SHOE_ON_RACK
# MANIP_TYPE = CategoryManipulationType.MUG_ON_SHELF_3D


EXPERIMENT_MODE = True

class GraspSupervisorState(object):
    STATUS_LIST = ["ABOVE_TABLE", "PRE_GRASP", "GRASP", "IK_FAILED", "NO_GRASP_FOUND", "GRASP_FOUND", "OBJECT_IN_GRIPPER", "GRASP_FAILED", "SAFETY_CHECK_FAILED", "PLANNING_FAILED", "FAILED"]

    def __init__(self):
        self.setPickFront()

        self.clear()

    def setPickFront(self):
        self.graspingLocation = "front"
        self.stowLocation = "left"

    def setPickLeft(self):
        self.graspingLocation = "left"
        self.stowLocation = "front"

    @property
    def grasp_data(self):
        return self._grasp_data

    @grasp_data.setter
    def grasp_data(self, value):
        """

        :param value: GraspData
        :return:
        """
        self._grasp_data = value

    @property
    def cache(self):
        return self._cache

    def clear(self):
        """
        Clear any stateful elements of the state
        :return:
        """
        self._grasp_data = None
        self._status = None
        self._cache = dict()
        self._trajectory_result = None

    def clear_cache(self):
        """
        Clears only the cache
        :return:
        """
        self._cache = dict()

    def set_status(self, status):
        assert status in GraspSupervisorState.STATUS_LIST
        self._status = status

    @property
    def status(self):
        return self._status

    @status.setter
    def status(self, status):
        assert status in GraspSupervisorState.STATUS_LIST
        self._status = status

    def set_status_ik_failed(self):
        self.status = "IK_FAILED"

    def print_status(self):
        """
        Prints the status
        :return:
        """
        if self._status is None:
            print "Current Status: None"
        else:
            print "Current Status: " + self._status


class GraspSupervisor(object):
    def __init__(self, graspingParamsFile=None, cameraSerialNumber="carmine_1", tfBuffer=None):
        self.graspingParamsFile = graspingParamsFile
        self.reloadParams()
        self.cameraSerialNumber = cameraSerialNumber

        self.cameraName = 'camera_' + str(cameraSerialNumber)
        self.pointCloudTopic = '/' + str(self.cameraName) + '/depth/points'
        self.rgbImageTopic = '/' + str(self.cameraName) + '/rgb/image_rect_color'
        self.depthImageTopic = '/' + str(self.cameraName) + '/depth_registered/sw_registered/image_rect'
        self.camera_info_topic = '/' + str(self.cameraName) + '/rgb/camera_info'
        self.graspFrameName = 'base'
        self.ggcnn_grasp_frame_camera_axes_id = "ggcnn_grasp"
        self.depthOpticalFrameName = self.cameraName + "_depth_optical_frame"
        self.rgbOpticalFrameName = self.cameraName + "_rgb_optical_frame"

        self.state = GraspSupervisorState()

        self.robotService = rosUtils.RobotService.makeKukaRobotService()
        self.robotService._use_debug_speed = USE_DEBUG_SPEED
        self.robotService._debug_speed = DEBUG_SPEED
        self.usingDirector = True
        self.tfBuffer = tfBuffer  # don't create a new one if it is passed in
        self.setupConfig()
        self._grasp_point = None  # stores the grasp point to be used in grasp3DLocation
        self._cache = dict()

        self._gripper = spartan.manipulation.gripper.Gripper.make_schunk_gripper()
        self._poser_visualizer = PoserVisualizer.make_default()
        self.poser_result = None
        self._object_manipulation = None

        self._category_manip = None # can be assigned later as needed

        self._shoe_manipulation_counter = 0


        filename = os.path.join(os.path.join(spartanUtils.getSpartanSourceDir(), 'src/catkin_projects/station_config/RLG_iiwa_1/stored_poses.yaml'))
        self._stored_poses_director = spartanUtils.getDictFromYamlFilename(filename)


        if USING_DIRECTOR:
            self.taskRunner = TaskRunner()
            self.taskRunner.callOnThread(self.setup)
        else:
            self.setup()

        self.debugMode = False
        if self.debugMode:
            print "\n\n----------WARNING GRASP SUPERVISOR IN DEBUG MODE----------\n"
            # if self.debugMode:
            #     self.pointCloudListMsg = GraspSupervisor.getDefaultPointCloudListMsg()

    def reloadParams(self):
        self.graspingParams = spartanUtils.getDictFromYamlFilename(self.graspingParamsFile)

    def setup(self):
        self.setupSubscribers()
        self.setupPublishers()
        self.setupTF()
        self.setupROSActions()
        self.gripperDriver = SchunkDriver()
        self.setup_visualization()

    def _clear_cache(self):
        """
        Clears our local cache of variables
        :return:
        """

        self._cache = dict()

    def setupDirector(self):
        self.taskRunner.callOnThread(self.setup)

    def setupConfig(self):
        self.config = dict()
        self.config['base_frame_id'] = "base"
        self.config['end_effector_frame_id'] = "iiwa_link_ee"
        self.config['pick_up_distance'] = 0.25  # distance to move above the table after grabbing the object

        self.config["sleep_time_for_sensor_collect"] = 0.1
        self.config['scan'] = dict()
        self.config['scan']['pose_list'] = ['scan_left_close', 'scan_above_table', 'scan_right']
        self.config['scan']['joint_speed'] = 45
        self.config['grasp_speed'] = 20

        normal_speed = 30
        self.config['speed'] = dict()
        self.config['speed']['stow'] = normal_speed
        self.config['speed']['pre_grasp'] = normal_speed
        self.config['speed']['grasp'] = 10

        self.config['home_pose_name'] = 'above_table_pre_grasp'
        self.config['grasp_nominal_direction'] = np.array([1, 0, 0])  # x forwards
        self.config['grasp_to_ee'] = dict()


        self.config["object_interaction"] = dict()
        self.config["object_interaction"]["speed"] = 10
        self.config["object_interaction"]["rotate_speed"] = 30
        self.config["object_interaction"]["pickup_distance"] = 0.15
        # self.config["object_interaction"]["drop_distance_above_grasp"] = 0.035 # good for shoes
        self.config["object_interaction"]["drop_distance_above_grasp"] = 0.002  # good for mugs
        self.config["object_interaction"]["drop_location"] = [0.65, 0, 0.5]  # z coordinate is overwritten later

        self.graspToIiwaLinkEE = spartanUtils.transformFromPose(
            self.graspingParams['gripper_palm_to_ee'])
        self.iiwaLinkEEToGraspFrame = self.graspToIiwaLinkEE.GetLinearInverse()

        self.gripper_fingertip_to_iiwa_link_ee = spartanUtils.transformFromPose(
            self.graspingParams['gripper_fingertip_to_ee'])
        self.T_gripper_fingertip__iiwa_link_ee = self.gripper_fingertip_to_iiwa_link_ee.GetLinearInverse()

        pos = [-0.15, 0, 0]
        quat = [1, 0, 0, 0]
        self.preGraspToGraspTransform = transformUtils.transformFromPose(pos, quat)

    def setupSubscribers(self):
        self.pointCloudSubscriber = rosUtils.SimpleSubscriber(self.pointCloudTopic, sensor_msgs.msg.PointCloud2)
        self.rgbImageSubscriber = rosUtils.SimpleSubscriber(self.rgbImageTopic, sensor_msgs.msg.Image)
        self.depthImageSubscriber = rosUtils.SimpleSubscriber(self.depthImageTopic, sensor_msgs.msg.Image)

        self.camera_info_subscriber = rosUtils.SimpleSubscriber(self.camera_info_topic, sensor_msgs.msg.CameraInfo)

        self.pointCloudSubscriber.start()
        self.rgbImageSubscriber.start()
        self.depthImageSubscriber.start()
        self.camera_info_subscriber.start()

        self.clicked_point_subscriber = rosUtils.SimpleSubscriber("/clicked_point", geometry_msgs.msg.PointStamped,
                                                                  self.on_clicked_point)
        self.clicked_point_subscriber.start()

        self.ggcnn_subscriber = rosUtils.SimpleSubscriber('ggcnn/out/command', std_msgs.msg.Float32MultiArray)

    def setupPublishers(self):
        """
        Sets up some ROS publishers
        """

        self.rviz_marker_publisher = rospy.Publisher("/spartan_grasp/visualization_marker",
                                                     visualization_msgs.msg.Marker, queue_size=1)

        self.rviz_marker_array_publisher = rospy.Publisher("/grasp_supervisor/visualization_marker_array",
                                                           visualization_msgs.msg.MarkerArray, queue_size=1)

        self.grasp_pointcloud_publisher = rospy.Publisher("/grasp_supervisor/points", sensor_msgs.msg.PointCloud2,
                                                          queue_size=1)

    def setup_visualization(self):
        self._vis_container = om.getOrCreateContainer("grasp supervisor")

    def on_clicked_point(self, clicked_point_msg):
        """
        Visualizes the clicked point in rviz
        """

        print "received a /clicked_point message . . . visualizing"
        pos = clicked_point_msg.point
        x, y, z = pos.x, pos.y, pos.z

        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clicked_point"
        marker.id = 0
        marker.type = visualization_msgs.msg.Marker.SPHERE
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # hack to get around director funny business
        for i in xrange(0, 5):
            self.rviz_marker_publisher.publish(marker)
            rospy.sleep(0.02)

    def get_clicked_point(self):
        """
        Returns the stored clicked point. If there is none it raises and error

        rtype: geometry_msgs.Point
        """
        lastMsg = self.clicked_point_subscriber.lastMsg
        if lastMsg is None:
            raise ValueError("No /clicked_point messages found.")

        return lastMsg.point

    def setupROSActions(self):

        actionName = '/spartan_grasp/GenerateGraspsFromPointCloudList'
        self.generate_grasps_client = actionlib.SimpleActionClient(actionName,
                                                                   spartan_grasp_msgs.msg.GenerateGraspsFromPointCloudListAction)

        actionName = '/spartan_grasp/Grasp3DLocation'
        self.grasp_3D_location_client = actionlib.SimpleActionClient(actionName,
                                                                     spartan_grasp_msgs.msg.Grasp3DLocationAction)

        findBestBatchActionName = '/FindBestMatch'
        self.find_best_match_client = actionlib.SimpleActionClient(findBestBatchActionName,
                                                                   pdc_ros_msgs.msg.FindBestMatchAction)

        poser_action_name = '/Poser'
        self.poser_client = actionlib.SimpleActionClient(poser_action_name,
                                                                   pdc_ros_msgs.msg.DeformableRegistrationAction)


        category_manipulation_name = "/CategoryManipulation"
        self.category_manip_client = actionlib.SimpleActionClient(category_manipulation_name, pdc_ros_msgs.msg.CategoryManipulationAction)

        action_name = "/KeypointDetection"
        self.keypoint_detection_client = actionlib.SimpleActionClient(action_name, pdc_ros_msgs.msg.KeypointDetectionAction)

        action_name = "/PoseEstimation"
        self.pose_estimation_client = actionlib.SimpleActionClient(action_name,
                                                                      pdc_ros_msgs.msg.EstimatePoseAction)

        action_name = "/SaveRGBD"
        self.save_RGBD_client = actionlib.SimpleActionClient(action_name,
                                                                      pdc_ros_msgs.msg.KeypointDetectionAction)

    def setupTF(self):
        if self.tfBuffer is None:
            self.tfBuffer = tf2_ros.Buffer()

        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

    def getDepthOpticalFrameToWorldTransform(self):
        depth_optical_frame_to_world = self.tfBuffer.lookup_transform("base", self.depthOpticalFrameName,
                                                                       rospy.Time(0))


        return depth_optical_frame_to_world

    def get_transform(self, from_name, to_name, ros_time=None):
        if ros_time is None:
            ros_time = rospy.Time(0)

        transform_stamped_msg = self.tfBuffer.lookup_transform(to_name, from_name, ros_time)
        # convert to vtkTransform
        pos, quat = rosUtils.poseFromROSTransformMsg(transform_stamped_msg.transform)

        return pos, quat


    def getRgbOpticalFrameToWorldTransform(self, time=None):
        """

        :param time:
        :type time:
        :return: geometry_msgs/TransformStamped
        :rtype:
        """
        if time is None:
            time = rospy.Time(0)

        rgb_optical_frame_to_world = self.tfBuffer.lookup_transform("base", self.rgbOpticalFrameName,
                                                                     time)

        return rgb_optical_frame_to_world



    def capturePointCloudAndCameraTransform(self, cameraOrigin=[0, 0, 0]):
        """
        Captures the current PointCloud2 from the sensor. Also records the pose of camera frame.
        """
        # sleep to transforms can update

        msg = spartan_grasp_msgs.msg.PointCloudWithTransform()
        msg.header.stamp = rospy.Time.now()

        msg.camera_origin.x = cameraOrigin[0]
        msg.camera_origin.y = cameraOrigin[1]
        msg.camera_origin.z = cameraOrigin[2]

        msg.point_cloud_to_base_transform = self.getDepthOpticalFrameToWorldTransform()

        msg.point_cloud = self.pointCloudSubscriber.waitForNextMessage()

        self.testData = msg  # for debugging
        return msg

    def captureRgbdAndCameraTransform(self, cameraOrigin=[0, 0, 0]):
        # sleep to transforms can update

        msg = pdc_ros_msgs.msg.RGBDWithPose()
        msg.header.stamp = rospy.Time.now()

        msg.camera_pose = self.getRgbOpticalFrameToWorldTransform()

        msg.rgb_image = self.rgbImageSubscriber.waitForNextMessage()
        msg.depth_image = self.depthImageSubscriber.waitForNextMessage()

        # maybe be careful about rostime here
        msg.point_cloud = self.pointCloudSubscriber.waitForNextMessage()
        msg.point_cloud_pose = self.getDepthOpticalFrameToWorldTransform()


        return msg


    def moveHome(self, speed=None):
        rospy.loginfo("moving home")
        if speed is None:
            speed = self.graspingParams['speed']['nominal']
        homePose = self.graspingParams[self.state.graspingLocation]['poses']['scan_above_table']
        self.robotService.moveToJointPosition(homePose,
                                              maxJointDegreesPerSecond=speed)

    def getStowPose(self):
        stow_location = self.state.stowLocation
        params = self.graspingParams[stow_location]
        return params['poses']['stow']

    # scans to several positions
    def collectSensorData(self, saveToBagFile=False, **kwargs):
        """
        Collects PointCloud Messages, also RGB and Depth images.

        Writes the result to two class variables
        - self.pointCloudListMsg 
        - self.listOfRgbdWithPose

        also returns these two values
        """
        self.moveHome()

        rospy.loginfo("collecting sensor data")
        graspLocationData = self.graspingParams[self.state.graspingLocation]

        pointCloudListMsg = spartan_grasp_msgs.msg.PointCloudList()
        pointCloudListMsg.header.stamp = rospy.Time.now()

        data = dict()
        pose_list = graspLocationData['scan_pose_list']
        listOfRgbdWithPoseMsg = []

        for poseName in pose_list:
            rospy.loginfo("moving to pose = " + poseName)
            joint_positions = graspLocationData['poses'][poseName]
            self.robotService.moveToJointPosition(joint_positions,
                                                  maxJointDegreesPerSecond=self.config['scan']['joint_speed'])

            rospy.sleep(self.config["sleep_time_for_sensor_collect"])
            pointCloudWithTransformMsg = self.capturePointCloudAndCameraTransform()
            pointCloudListMsg.point_cloud_list.append(pointCloudWithTransformMsg)
            data[poseName] = pointCloudWithTransformMsg

            rgbdWithPoseMsg = self.captureRgbdAndCameraTransform()
            listOfRgbdWithPoseMsg.append(rgbdWithPoseMsg)

        self.sensorData = data
        self.pointCloudListMsg = pointCloudListMsg
        self.listOfRgbdWithPoseMsg = listOfRgbdWithPoseMsg

        if saveToBagFile:
            self.saveSensorDataToBagFile(pointCloudListMsg=pointCloudListMsg, **kwargs)

        return pointCloudListMsg, listOfRgbdWithPoseMsg


    def findBestBatch(self):
        """
        This function will:
        - collect a small handful of RGBDWithPose msgs
        - call the FindBestMatch service (a service of pdc-ros)
        - return what was found from FindBestMatch
        """
        self.moveHome()
        _, listOfRgbdWithPoseMsg = self.collectSensorData()
        self.list_rgbd_with_pose_msg = listOfRgbdWithPoseMsg

        # request via a ROS Action
        rospy.loginfo("waiting for find best match server")
        self.find_best_match_client.wait_for_server()

        goal = pdc_ros_msgs.msg.FindBestMatchGoal()
        goal.rgbd_with_pose_list = listOfRgbdWithPoseMsg
        goal.camera_info = self.camera_info_subscriber.waitForNextMessage()

        rospy.loginfo("requesting best match from server")

        self.find_best_match_client.send_goal(goal)
        self.moveHome()

        rospy.loginfo("waiting for find best match result")
        self.find_best_match_client.wait_for_result()
        result = self.find_best_match_client.get_result()
        rospy.loginfo("received best match result")

        self.best_match_result = result

        if result.match_found:
            print "match found"
            print "location:", result.best_match_location
        else:
            print "NO MATCH FOUND"

        return result

    def run_poser(self):
        """
        This function will:
        - collect a small handful of RGBDWithPose msgs
        - call the FindBestMatch service (a service of pdc-ros)
        - return what was found from FindBestMatch
        """
        # self.moveHome()



        rgbdWithPoseMsg = self.captureRgbdAndCameraTransform()
        listOfRgbdWithPoseMsg = [rgbdWithPoseMsg]
        self.list_rgbd_with_pose_msg = listOfRgbdWithPoseMsg

        # request via a ROS Action
        rospy.loginfo("waiting for poser server")
        self.poser_client.wait_for_server()
        rospy.loginfo("connected to poser server")

        goal = pdc_ros_msgs.msg.DeformableRegistrationGoal()
        goal.rgbd_with_pose_list = listOfRgbdWithPoseMsg
        goal.camera_info = self.camera_info_subscriber.waitForNextMessage()

        rospy.loginfo("requesting registration from poser")

        self.poser_client.send_goal(goal)
        self.moveHome()

        rospy.loginfo("waiting for poser result")
        self.poser_client.wait_for_result()
        result = self.poser_client.get_result()
        state = self.poser_client.get_state()
        rospy.loginfo("received poser result")

        print("result:\n", result)

        succeeded = (state == GoalStatus.SUCCEEDED)

        if not succeeded:
            rospy.loginfo("Poser failed")

        self.poser_result = result
        self._cache['poser_result'] = result

        result_dict = dict()
        result_dict['result'] = result
        result_dict['output_dir'] = result.output_dir
        result_dict['state'] = state
        result_dict['succeeded'] = succeeded
        result_dict['type'] = "mankey"
        self._cache["keypoint_detection_result"] = result_dict

        self.taskRunner.callOnMain(self.visualize_poser_result)

    def run_keypoint_detection(self, wait_for_result=True, move_to_stored_pose=True, clear_state=True):
        """
        Runs keypoint detection using ManKey in pdc-ros. Note that this clears the cache
        :return:
        :rtype:
        """

        if clear_state:
            self._clear_cache()
            self.state.clear()



        if move_to_stored_pose:

            CMT = CategoryManipulationType
            q = self._stored_poses_director["General"]["home"]  # for mugs
            if MANIP_TYPE in [CMT.SHOE_ON_RACK, CMT.SHOE_ON_TABLE]:
                q = self._stored_poses_director['General']['center_back']
            else:  # basically all mugs
                q = self._stored_poses_director["General"]["home"]

            self.robotService.moveToJointPosition(q,
                                                  maxJointDegreesPerSecond=self.graspingParams['speed']['fast'])

        rgbdWithPoseMsg = self.captureRgbdAndCameraTransform()
        self.state.cache['rgbd_with_pose_list'] = []
        self.state.cache['rgbd_with_pose_list'].append(rgbdWithPoseMsg)

        # request via a ROS Action
        rospy.loginfo("waiting for KeypointDetection server")
        self.keypoint_detection_client.wait_for_server()
        rospy.loginfo("connected to KeypointDetection server")

        goal = pdc_ros_msgs.msg.KeypointDetectionGoal()
        goal.rgbd_with_pose_list = self.state.cache['rgbd_with_pose_list']
        goal.camera_info = self.camera_info_subscriber.waitForNextMessage()

        if EXPERIMENT_MODE:
            goal.output_dir = "mankey_experiments/%s" %(spartanUtils.get_current_YYYY_MM_DD_hh_mm_ss())

        rospy.loginfo("requesting action from KeypointDetection server")

        self.keypoint_detection_client.send_goal(goal)
        self.state.set_status("ABOVE_TABLE")

        if wait_for_result:
            self.wait_for_keypoint_detection_result()        

    def wait_for_keypoint_detection_result(self):
        """
        Wait for keypont detection result, save it to cache
        """
        rospy.loginfo("waiting for KeypointDetection result")
        self.keypoint_detection_client.wait_for_result()
        result = self.keypoint_detection_client.get_result()
        state = self.keypoint_detection_client.get_state()
        rospy.loginfo("received KeypointDetection result")
        print "result:\n", result

        self.keypoint_detection_result = result

        succeeded = (state == GoalStatus.SUCCEEDED)

        if not succeeded:
            rospy.loginfo("KeypointDetection failed")

        result_dict = dict()
        result_dict['result'] = result
        result_dict['output_dir'] = result.output_dir
        result_dict['state'] = state
        result_dict['succeeded'] = succeeded
        result_dict['type'] = "mankey"
        self._cache["keypoint_detection_result"] = result_dict

        self.state._cache["keypoint_detection_result"] = result_dict

        return result_dict

    def check_keypoint_detection_succeeded(self):
        """
        Checks whether keypoint detection succeeded or not
        :return:
        :rtype:
        """

        # you should have run keypoint detection before this
        keypoint_detection_result = self.state.cache['keypoint_detection_result']
        if keypoint_detection_result["state"] == GoalStatus.SUCCEEDED:
            return True
        else:
            print("keypoint detection failed, ABORTING")
            return False


    def check_category_goal_estimation_succeeded(self):
        """
        Returns a bool as to whether category goal estimation succeeded or not
        :return:
        :rtype:
        """

        state = self.state.cache['category_manipulation_goal']['state']
        if state == GoalStatus.SUCCEEDED:
            return True
        else:
            print("category goal estimation failed, ABORTING")
            return False

    def estimate_mug_rack_pose(self):
        """

        :return:
        :rtype:
        """

        # fusion_params_file = os.path.join(spartanUtils.getSpartanSourceDir(), "src/catkin_projects/station_config/RLG_iiwa_1/fusion/fusion_params.yaml")
        #
        #
        # fusion_params = spartanUtils.getDictFromYamlFilename(fusion_params_file)
        # bbox_min = np.array(fusion_params['left']['bbox_min'])
        # bbox_min[2] += 0.05 # be conservative on where bottom of table is
        # bbox_max = np.array(fusion_params['left']['bbox_max'])

        bbox_min = np.array([0.07001, 0.49, 0.01026])
        bbox_max = np.array([0.47195, 0.85201, 0.75])

        rgbd_with_pose_list = []

        # move to pose 1, capture RGBD
        q = self._stored_poses_director["left_table"]["look_at_rack"]
        speed = self.graspingParams["speed"]["fast"]
        self.robotService.moveToJointPosition(q, maxJointDegreesPerSecond=speed)

        rgbd_with_pose = self.captureRgbdAndCameraTransform()
        rgbd_with_pose_list.append(rgbd_with_pose)


        # move to pose 2, capture RGBD
        q = self._stored_poses_director["left_table"]["look_at_rack_2"]
        speed = self.graspingParams["speed"]["fast"]
        self.robotService.moveToJointPosition(q, maxJointDegreesPerSecond=speed)

        rgbd_with_pose = self.captureRgbdAndCameraTransform()
        rgbd_with_pose_list.append(rgbd_with_pose)

        # convert to VTK poly data and crop
        d = DebugData()
        for msg in rgbd_with_pose_list:
            pointcloud_numpy = DirectorROSVisualizer.numpy_from_pointcloud2_msg(msg.point_cloud)
            pointcloud_vtk = vnp.getVtkPolyDataFromNumpyPoints(pointcloud_numpy)

            T_world_pointcloud = ros_numpy.numpify(msg.point_cloud_pose.transform)
            T_world_pointcloud_vtk = transformUtils.getTransformFromNumpy(T_world_pointcloud)
            pointcloud_vtk = filterUtils.transformPolyData(pointcloud_vtk, T_world_pointcloud_vtk)
            d.addPolyData(pointcloud_vtk)

        pointcloud = d.getPolyData()

        print "pointcloud.GetNumberOfPoints()", pointcloud.GetNumberOfPoints()

        # crop
        transform = vtk.vtkTransform()
        bounds = np.zeros([2,3])
        bounds[0,:] = bbox_min
        bounds[1,:] = bbox_max

        print "bounds", bounds

        cropped_pointcloud = segmentation.cropToBounds(pointcloud, transform, bounds)

        print "cropped_pointcloud.GetNumberOfPoints()", cropped_pointcloud.GetNumberOfPoints()


        # visualize it
        def vis_function():
            print "visualizing pointcloud"
            vis.showPolyData(pointcloud, "pointcloud")
            vis.showPolyData(cropped_pointcloud, "Mug rack pointcloud")


        self.mug_rack_pointcloud = cropped_pointcloud


        # not working for some reason
        print "visualizing"
        self.taskRunner.callOnMain(vis_function)

        return


        rgbd_with_pose = pdc_ros_msgs.msg.RGBDWithPose()

        # N x 3
        cropped_pointcloud_numpy = vnp.getNumpyFromVtk(cropped_pointcloud)
        print "cropped_pointcloud_numpy.shape", cropped_pointcloud_numpy.shape

        # save numpy to file
        save_file = "/home/manuelli/sandbox/spartan/pointcloud.npy"
        np.save(save_file, cropped_pointcloud_numpy)


        return
        # it's already in world frame
        rgbd_with_pose.point_cloud = DirectorROSVisualizer.pointcloud2_msg_from_numpy(cropped_pointcloud_numpy)


        # convert it back to ROS msg
        goal = pdc_ros_msgs.msg.EstimatePoseGoal()
        goal.rgbd_with_pose_list.append(rgbd_with_pose)


        T_world_rack_vtk = self._category_manip.mug_rack_vis_obj.getChildFrame().transform
        T_world_rack = transformUtils.getNumpyFromTransform(T_world_rack_vtk)
        goal.T_init = ros_numpy.msgify(geometry_msgs.Pose, T_world_rack)


        # send out service call
        self.pose_estimation_client.wait_for_server()
        self.pose_estimation_client.send_goal(goal)

        # wait for result
        self.pose_estimation_client.wait_for_result()
        result = self.pose_estimation_client.get_result()


        T_world_rack_estimated = ros_numpy.numpify(result.T_world_model)
        T_world_rack_estimated_vtk = transformUtils.getTransformFromNumpy(T_world_rack_estimated)
        self._category_manip.mug_rack_vis_obj.getChildFrame().copyFrame(T_world_rack_estimated_vtk)


    def run_category_manipulation_goal_estimation(self, wait_for_result=True, capture_rgbd=True):
        """
        Calls the CategoryManipulation service of pdc-ros
        which is provided by category_manip_server.py.

        Uses the keypoint detection result from either

        `run_poser` or `run_keypoint_detection`
        :return: bool
        :rtype:
        """

        if not self.check_keypoint_detection_succeeded():
            return False

        keypoint_detection_result = self.state.cache['keypoint_detection_result']

        # don't specify poser output dir for now
        goal = pdc_ros_msgs.msg.CategoryManipulationGoal()
        goal.output_dir = keypoint_detection_result['output_dir']
        goal.keypoint_detection_type = keypoint_detection_result['type']



        if capture_rgbd:
            self.moveHome()
            rgbd_with_pose = self.captureRgbdAndCameraTransform()
            self.state.cache['rgbd_with_pose_list'].append(rgbd_with_pose)
            goal.rgbd_with_pose_list = self.state.cache['rgbd_with_pose_list']


        if 'rgbd_with_pose_list' in self.state.cache:
            goal.rgbd_with_pose_list = self.state.cache['rgbd_with_pose_list']

        if MANIP_TYPE == CategoryManipulationType.SHOE_ON_RACK:
            print("applying T_adjust")
            print("self._shoe_manipulation_counter", self._shoe_manipulation_counter)
            goal.apply_T_adjust = True
            pos = np.array([self.graspingParams["shoe_offset"], 0, 0]) * self._shoe_manipulation_counter
            quat = [1,0,0,0]
            T_adjust_vtk = transformUtils.transformFromPose(pos, quat)
            T_adjust = transformUtils.getNumpyFromTransform(T_adjust_vtk)
            goal.T_adjust = ros_numpy.msgify(geometry_msgs.msg.Pose, T_adjust)
        else:
            goal.apply_T_adjust =False

        rospy.loginfo("waiting for CategoryManip server")
        self.category_manip_client.wait_for_server()
        rospy.loginfo("connected to CategoryManip server")

        self.category_manip_client.send_goal(goal)
        
        if wait_for_result:
            self.wait_for_category_manipulation_goal_result()

        return True

    def wait_for_category_manipulation_goal_result(self):
        """
        Waits for category manipulation goal result
        """

        print("waiting for category manipulation result")
        self.category_manip_client.wait_for_result()
        result = self.category_manip_client.get_result()
        state = self.category_manip_client.get_state()


        T_goal_obs = ros_numpy.numpify(result.T_goal_obs)

        print "T_goal_obs:\n", T_goal_obs

        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)
        print transformUtils.poseFromTransform(T_goal_obs_vtk)


        self.state.cache['category_manipulation_goal'] = dict()
        self.state.cache['category_manipulation_goal']['result'] = result
        self.state.cache['category_manipulation_goal']["T_goal_obs"] = T_goal_obs_vtk
        self.state.cache['category_manipulation_goal']['state'] = state
        self.state.cache['category_manipulation_goal']["type"] = CategoryManipulationType.from_string(result.category_manipulation_type)

    def run_mug_shelf_3D_pipeline(self):
        """
        Runs entire pipeline for mug shelf 3D
        :return:
        :rtype:
        """

        self.state.clear()
        self._clear_cache()

        # move home
        speed = self.graspingParams['speed']['fast']
        super_fast_speed = self.graspingParams['speed']['fast']
        # q = self._stored_poses_director["General"]["home"]
        # q = self._stored_poses_director["mug"]["image_capture_for_mug_shelf"]
        q = self._stored_poses_director["General"]["center_back"]
        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=super_fast_speed)

        self.run_keypoint_detection(wait_for_result=False, move_to_stored_pose=False, clear_state=False)

        # run keypoint detection
        # move to center back to capture another RGBD image
        q = self._stored_poses_director["General"]["home"]
        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=super_fast_speed)

        rgbd_with_pose = self.captureRgbdAndCameraTransform()
        self.state.cache['rgbd_with_pose_list'].append(rgbd_with_pose)

        self.wait_for_keypoint_detection_result()

        if not self.check_keypoint_detection_succeeded():
            self.state.set_status("FAILED")
            return False

        # run category manip
        code = self.run_category_manipulation_goal_estimation(capture_rgbd=False)
        if not code:
            self.state.set_status("FAILED")
            return False


        self.wait_for_category_manipulation_goal_result()
        if not self.check_category_goal_estimation_succeeded():
            self.state.set_status("PLANNING_FAILED")
            return False

        # run the manipulation
        # need safety checks in there before running autonomously
        code = self.run_mug_shelf_manipulation()
        if not (code == True):
            self.state.set_status("FAILED")
            return False

        # if the place was successful then retract
        self.retract_from_mug_shelf()

        if EXPERIMENT_MODE:
            output_dir = self.state.cache['keypoint_detection_result']['output_dir']
            print "\n\n", os.path.split(output_dir)[1]

    def run_mug_on_rack_pipeline(self, side_view=False):
        """
        Runs entire pipeline for mug shelf 3D
        :return:
        :rtype:
        """

        self.state.clear()
        self._clear_cache()

        # move home
        speed = self.graspingParams['speed']['fast']


        q = self._stored_poses_director["General"]["home"]
        if side_view:
            print "\nusing side view\n"
            q = self._stored_poses_director["General"]["center_back"]

        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=speed)

        # run keypoint detection
        self.run_keypoint_detection(wait_for_result=False, move_to_stored_pose=False, clear_state=False)
        self.wait_for_keypoint_detection_result()


        # move to center back to capture another RGBD image

        q = self._stored_poses_director["General"]["center_back"]
        if side_view:
            q = self._stored_poses_director["General"]["home"]
        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=speed)

        rgbd_with_pose = self.captureRgbdAndCameraTransform()
        self.state.cache['rgbd_with_pose_list'].append(rgbd_with_pose)

        q = self._stored_poses_director["General"]["home"]
        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=speed)


        if not self.check_keypoint_detection_succeeded():
            self.state.set_status("FAILED")
            return False

        # run category manip
        code = self.run_category_manipulation_goal_estimation(capture_rgbd=False)
        if not code:
            self.state.set_status("FAILED")
            return False


        self.wait_for_category_manipulation_goal_result()
        if not self.check_category_goal_estimation_succeeded():
            self.state.set_status("PLANNING_FAILED")
            return False

        
        # run the manipulation
        # need safety checks in there before running autonomously
        code = self.run_mug_on_rack_manipulation()
        if not (code == True):
            self.state.set_status("FAILED")
            return False


        if EXPERIMENT_MODE:
            output_dir = self.state.cache['keypoint_detection_result']['output_dir']
            print "\n\n", os.path.split(output_dir)[1]

    def run_shoe_on_rack_pipeline(self):
        """
        Runs entire pipeline for mug shelf 3D
        :return:
        :rtype:
        """

        if EXPERIMENT_MODE:
            self._shoe_manipulation_counter = 0 # for testing

        self.state.clear()
        self._clear_cache()

        # move home
        speed = self.graspingParams['speed']['fast']
        # q = self._stored_poses_director["General"]["center_back"]
        q = self._stored_poses_director["General"]["home"]
        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=speed)

        # run keypoint detection
        self.run_keypoint_detection(wait_for_result=False, move_to_stored_pose=False, clear_state=False)
        self.wait_for_keypoint_detection_result()

        if not self.check_keypoint_detection_succeeded():
            self.state.set_status("FAILED")
            return False

        # run category manip
        code = self.run_category_manipulation_goal_estimation(capture_rgbd=False)
        if not code:
            self.state.set_status("FAILED")
            return False


        self.wait_for_category_manipulation_goal_result()
        if not self.check_category_goal_estimation_succeeded():
            self.state.set_status("PLANNING_FAILED")
            return False



        # run the manipulation
        # need safety checks in there before running autonomously
        code = self.run_shoe_rack_manipulation()
        if not code:
            self.state.set_status("FAILED")
            return False

        # if the place was successful then retract
        self.retract_from_shoe_rack()

        if EXPERIMENT_MODE:
            print "\n\n", self.state.cache['keypoint_detection_result']['output_dir']

    def run_manipulate_object(self, debug=False):
        """
        Runs the object manipulation code. Will put the object into the
        specified target pose from `run_category_manipulation_goal_estimation`
        :return:
        """

        # self.taskRunner.callOnMain(self._poser_visualizer.visualize_result)

        if not self.check_category_goal_estimation_succeeded():
            return False

        if debug:
            self._object_manipulation = ObjectManipulation()
            self._object_manipulation.assign_defaults()
            self._object_manipulation.compute_transforms()
            return

        self.moveHome()
        grasp_found, grasp_data = self.request_spartan_grasp(clear_state=False)
        if not grasp_found:
            print "no grasp found, returning\n"
            return False


        # execute the grasp
        object_in_gripper = self.execute_grasp(self.state.grasp_data, close_gripper=True, use_cartesian_plan=True)

        print "object_in_gripper:", object_in_gripper

        T_goal_obs = self.state.cache['category_manipulation_T_goal_obs']
        T_W_G = self.state.cache['gripper_frame_at_grasp']
        self._object_manipulation = ObjectManipulation(T_goal_object=T_goal_obs, T_W_G=T_W_G)
        self._object_manipulation.grasp_data = self.state.grasp_data
        self._object_manipulation.compute_transforms()

        self.taskRunner.callOnMain(self._object_manipulation.visualize)


        pre_grasp_pose = self.state.cache['pre_grasp_ik_response'].joint_state.position
        pickup_speed = self.graspingParams['speed']['pickup']

        if not object_in_gripper:
            # open the gripper and back away
            self.gripperDriver.send_open_gripper_set_distance_from_current()
            self.robotService.moveToJointPosition(pre_grasp_pose,
                                                   maxJointDegreesPerSecond=
                                                  pickup_speed)
            return False

        # pickup the object
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pickup_speed)


        # place the object
        grasp_data_place = self._object_manipulation.get_place_grasp_data()
        self.execute_place(grasp_data_place)


        # open the gripper and back away
        pre_grasp_pose = self.state.cache['pre_grasp_ik_response'].joint_state.position
        pickup_speed = self.graspingParams['speed']['pickup']
        self.gripperDriver.send_open_gripper_set_distance_from_current()
        # pickup the object
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pickup_speed)

        # move home
        self.moveHome()

    def run_shoe_rack_manipulation(self, debug=False, push_in_distance=0.00):
        """
        Runs the object manipulation code. Will put the object into the
        specified target pose from `run_category_manipulation_goal_estimation`
        :return:
        """

        print("\n\n--- Running Shoe Manipulation-------\n\n")

        # self.taskRunner.callOnMain(self._poser_visualizer.visualize_result)

        if not self.check_category_goal_estimation_succeeded():
            return False

        # check that we really are doing mug
        category_manipulation_type = self.state.cache['category_manipulation_goal']['type']
        assert category_manipulation_type == CategoryManipulationType.SHOE_ON_RACK


        speed = self.graspingParams['speed']['fast']
        self.moveHome(speed=speed)

        result = self.state.cache['category_manipulation_goal']['result']
        T_W_fingertip = ros_numpy.numpify(result.T_world_gripper_fingertip)
        T_W_fingertip_vtk = transformUtils.getTransformFromNumpy(T_W_fingertip)

        grasp_data = GraspData.from_gripper_fingertip_frame(T_W_fingertip)
        grasp_data.gripper.params["hand_inner_diameter"] = result.gripper_width
        grasp_data.gripper.params["hand_inner_diameter"] = 0.07
        self.state.grasp_data = grasp_data

        # rotate the grasp to align with nominal
        params = self.getParamsForCurrentLocation()
        grasp_z_axis_nominal = np.array(params['grasp']['grasp_nominal_direction'])
        grasp_data.rotate_grasp_frame_to_nominal(grasp_z_axis_nominal)




        def vis_function():
            vis.updateFrame(T_W_fingertip_vtk, "gripper fingertip frame", scale=0.15, parent=self._vis_container)

            vis.updateFrame(grasp_data.grasp_frame, "grasp frame", scale=0.15, parent=self._vis_container)

            self.visualize_grasp(grasp_data)

        self.taskRunner.callOnMain(vis_function)

        # execute the grasp
        force_threshold_magnitude = 30
        object_in_gripper = self.execute_grasp(grasp_data, close_gripper=True, use_cartesian_plan=True, force_threshold_magnitude=force_threshold_magnitude, push_in_distance=0.04, ee_speed_m_s=0.1)


        if not object_in_gripper:
            print("grasp failed, returning")
            return False

        print "object_in_gripper:", object_in_gripper

        T_goal_obs = self.state.cache['category_manipulation_goal']["T_goal_obs"]
        T_W_G = self.state.cache['gripper_frame_at_grasp']



        pre_grasp_pose = self.state.cache['pre_grasp_ik_response'].joint_state.position
        pickup_speed = self.graspingParams['speed']['pickup']

        if not object_in_gripper:
            # open the gripper and back away
            self.gripperDriver.send_open_gripper_set_distance_from_current()
            self.robotService.moveToJointPosition(pre_grasp_pose,
                                                   maxJointDegreesPerSecond=
                                                  pickup_speed)
            return False

        # pickup the object
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pickup_speed)

        # move home
        self.moveHome()

        # move to approach pose
        speed = self.graspingParams['speed']['fast']
        q_approach = np.array(self._stored_poses_director["left_table"]["shoe_approach"])
        self.robotService.moveToJointPosition(q_approach, maxJointDegreesPerSecond=speed)


        # compute some poses
        T_goal_obs = ros_numpy.numpify(result.T_goal_obs)  # 4 x 4 numpy matrix
        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)
        object_manip = ObjectManipulation(T_goal_object=T_goal_obs_vtk, T_W_G=T_W_G)
        object_manip.compute_transforms()
        T_W_Gn_vtk = object_manip.T_W_Gn  # gripper to world for place pose

        T_pre_goal_obs = ros_numpy.numpify(result.T_pre_goal_obs)
        T_pre_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_pre_goal_obs)
        object_manip_approach = ObjectManipulation(T_goal_object=T_pre_goal_obs_vtk, T_W_G=T_W_G)
        object_manip_approach.compute_transforms()
        T_W_Gn_approach_vtk = object_manip_approach.T_W_Gn

        # move this down by push_in_distance

        pos, quat = transformUtils.poseFromTransform(T_W_Gn_approach_vtk)
        T_W_Gn_approach_vtk = transformUtils.transformFromPose(pos, quat)


        # now convert these to ee poses for running IK
        pos, quat = transformUtils.poseFromTransform(T_W_Gn_vtk)
        pos[2] -= push_in_distance
        T_W_Gn_vtk = transformUtils.transformFromPose(pos, quat)
        T_W_ee_vtk = self.getIiwaLinkEEFrameFromGraspFrame(T_W_Gn_vtk)
        T_W_ee = transformUtils.getNumpyFromTransform(T_W_ee_vtk)

        T_W_ee_approach_vtk = self.getIiwaLinkEEFrameFromGraspFrame(T_W_Gn_approach_vtk)
        T_W_ee_approach = transformUtils.getNumpyFromTransform(T_W_ee_approach_vtk)


        # place the object
        force_threshold_magnitude = 50 # shoes are heavy
        q_nom = np.array(self._stored_poses_director["Grasping"]["above_table_pre_grasp"])
        q_nom = np.array(self._stored_poses_director["left_table"]["above_table_pre_grasp"])
        code =self.execute_place_new(T_W_ee, T_W_ee_approach, q_nom=q_nom, use_cartesian_plan=True, force_threshold_magnitude=force_threshold_magnitude)

        print("\n\n--- Finished Shoe Manipulation-------\n\n")

        self._shoe_manipulation_counter += 1

        return code



    def retract_from_shoe_rack(self):
        """
        Retract from the shoe rack
        :return:
        :rtype:
        """

        # open the gripper and back away
        self.gripperDriver.send_open_gripper_set_distance_from_current(distance=0.045)

        # back away along gripper x-direction
        ee_speed_m_s = 0.05
        xyz_goal = [-0.15, 0, 0]  # 10 cm
        duration = np.linalg.norm(xyz_goal) / ee_speed_m_s
        ee_frame_id = "iiwa_link_ee"
        base_frame_id = "base"
        expressed_in_frame = ee_frame_id
        cartesian_traj_goal = \
            control_utils.make_cartesian_trajectory_goal(xyz_goal,
                                                         ee_frame_id,
                                                         expressed_in_frame,
                                                         duration=duration,
                                                         speed=0.1)

        action_client = self.robotService.cartesian_trajectory_action_client
        action_client.send_goal(cartesian_traj_goal)

        # wait for result
        action_client.wait_for_result()
        result = action_client.get_result()
        self.state.cache['cartesian_traj_result'] = result

        speed = self.graspingParams['speed']['fast']


        if EXPERIMENT_MODE:
            # move to pose
            q = self._stored_poses_director["left_table"]["shoe_evaluation_side"]
            self.robotService.moveToJointPosition(q, maxJointDegreesPerSecond=speed)
            msg = self.captureRgbdAndCameraTransform()
            save_dir = os.path.join(spartanUtils.get_sandbox_dir(), self.state.cache['keypoint_detection_result']['output_dir'], "evaluation")

            self.save_RGBD_client.wait_for_server()

            goal = pdc_ros_msgs.msg.KeypointDetectionGoal()
            goal.rgbd_with_pose_list.append(msg)
            goal.camera_info = self.camera_info_subscriber.waitForNextMessage()
            goal.output_dir = save_dir

            self.save_RGBD_client.send_goal(goal)
            self.save_RGBD_client.wait_for_result()

        self.moveHome(speed=speed)

    def run_mug_on_rack_manipulation(self):
        """
        Runs the object manipulation code. Will put the object into the
        specified target pose from `run_category_manipulation_goal_estimation`
        :return:
        """

        self.wait_for_category_manipulation_goal_result()

        if not self.check_category_goal_estimation_succeeded():
            return False

        category_manipulation_type = self.state.cache['category_manipulation_goal']['type']
        assert category_manipulation_type == CategoryManipulationType.MUG_ON_RACK

        self.moveHome()

        # extract grasp from gripper fingertip pose
        result = self.state.cache["category_manipulation_goal"]["result"]

        T_W_fingertip = ros_numpy.numpify(result.T_world_gripper_fingertip)
        T_W_fingertip_vtk = transformUtils.getTransformFromNumpy(T_W_fingertip)

        grasp_data = GraspData.from_gripper_fingertip_frame(T_W_fingertip)
        grasp_data.gripper.params["hand_inner_diameter"] = 0.05 # 4 cm wide

        self.state.grasp_data = grasp_data
        self.visualize_grasp(grasp_data)

        debug_speed = 10

        def vis_function():
            vis.updateFrame(T_W_fingertip_vtk, "gripper fingertip frame", scale=0.15, parent=self._vis_container)

            vis.updateFrame(grasp_data.grasp_frame, "grasp frame", scale=0.15, parent=self._vis_container)

        self.taskRunner.callOnThread(vis_function)

        # debugging
        print("visualizing grasp")
        self.visualize_grasp(grasp_data)

        # execute the grasp
        object_in_gripper = self.execute_grasp(self.state.grasp_data, close_gripper=True, use_cartesian_plan=True, push_in_distance=0.01, ee_speed_m_s=0.1)

        T_W_G = self.state.cache['gripper_frame_at_grasp'] # this is set in execute_grasp

        pre_grasp_pose = self.state.cache['pre_grasp_ik_response'].joint_state.position
        pickup_speed = self.graspingParams['speed']['pickup']

        if not object_in_gripper:
            # open the gripper and back away
            self.gripperDriver.send_open_gripper_set_distance_from_current()
            self.robotService.moveToJointPosition(pre_grasp_pose,
                                                   maxJointDegreesPerSecond=
                                                  pickup_speed)
            return False

        # pickup the object
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pickup_speed)



        # now move to nominal position for the place
        # speed = self.graspingParams["speed"]["nominal"]
        speed = self.graspingParams["speed"]["fast"]
        # q_nom_left_table = self._stored_poses_director["left_table"]["above_table_pre_grasp"]
        q_nom_left_table = self._stored_poses_director["left_table"]["above_table_pre_grasp_right"]
        self.robotService.moveToJointPosition(q_nom_left_table,
                                              maxJointDegreesPerSecond=
                                              speed)





        # compute some poses
        T_goal_obs = ros_numpy.numpify(result.T_goal_obs)  # 4 x 4 numpy matrix
        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)
        object_manip = ObjectManipulation(T_goal_object=T_goal_obs_vtk, T_W_G=T_W_G)
        object_manip.compute_transforms()
        T_W_Gn_vtk = object_manip.T_W_Gn # gripper to world for place pose


        T_pre_goal_obs = ros_numpy.numpify(result.T_pre_goal_obs)
        T_pre_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_pre_goal_obs)
        object_manip_approach = ObjectManipulation(T_goal_object=T_pre_goal_obs_vtk, T_W_G=T_W_G)
        object_manip_approach.compute_transforms()
        T_W_Gn_approach_vtk = object_manip_approach.T_W_Gn


        # now convert these to ee poses
        T_W_ee_vtk = self.getIiwaLinkEEFrameFromGraspFrame(T_W_Gn_vtk)
        T_W_ee = transformUtils.getNumpyFromTransform(T_W_ee_vtk)

        T_W_ee_approach_vtk = self.getIiwaLinkEEFrameFromGraspFrame(T_W_Gn_approach_vtk)
        T_W_ee_approach = transformUtils.getNumpyFromTransform(T_W_ee_approach_vtk)


        # execute the place
        print("executing place on rack")
        return self.execute_place_new(T_W_ee, T_W_ee_approach, q_nom=q_nom_left_table, use_cartesian_plan=True, force_threshold_magnitude=30, ee_speed_m_s=0.1)


    def retract_from_mug_rack(self, gripper_open=True):
        """
        Move backwards from the mug rack
        :return:
        :rtype:
        """

        category_manipulation_type = self.state.cache['category_manipulation_goal']['type']
        assert category_manipulation_type == CategoryManipulationType.MUG_ON_RACK

        if gripper_open:
            self.gripperDriver.send_open_gripper_set_distance_from_current()

        xyz_goal = np.array([-0.10, 0, 0])
        ee_frame_id = "iiwa_link_ee"
        expressed_in_frame = ee_frame_id
        cartesian_grasp_speed = self.graspingParams['speed']['cartesian_grasp']
        cartesian_traj_goal = \
            control_utils.make_cartesian_trajectory_goal(xyz_goal,
                                                         ee_frame_id,
                                                         expressed_in_frame,
                                                         speed=cartesian_grasp_speed)

        action_client = self.robotService.cartesian_trajectory_action_client
        action_client.send_goal(cartesian_traj_goal)

        # wait for result
        action_client.wait_for_result()
        result = action_client.get_result()

        # now move to nominal position for the place
        speed = self.graspingParams["speed"]["fast"]
        super_fast_speed = self.graspingParams["speed"]["super_fast"]
        # q_nom_left_table = self._stored_poses_director["left_table"]["above_table_pre_grasp"]
        q_nom_left_table = self._stored_poses_director["left_table"]["above_table_pre_grasp_right"]
        self.robotService.moveToJointPosition(q_nom_left_table,
                                              maxJointDegreesPerSecond=
                                              speed)



        if EXPERIMENT_MODE:
            q = self._stored_poses_director["left_table"]["mug_rack_evaluation"]
            self.robotService.moveToJointPosition(q,
                                                  maxJointDegreesPerSecond=
                                                  speed)

            msg = self.captureRgbdAndCameraTransform()
            save_dir = os.path.join(spartanUtils.get_sandbox_dir(),
                                    self.state.cache['keypoint_detection_result']['output_dir'], "evaluation")

            self.save_RGBD_client.wait_for_server()

            goal = pdc_ros_msgs.msg.KeypointDetectionGoal()
            goal.rgbd_with_pose_list.append(msg)
            goal.camera_info = self.camera_info_subscriber.waitForNextMessage()
            goal.output_dir = save_dir

            self.save_RGBD_client.send_goal(goal)
            self.save_RGBD_client.wait_for_result()


        self.moveHome(speed=super_fast_speed)

        if EXPERIMENT_MODE:
            output_dir = self.state.cache['keypoint_detection_result']['output_dir']
            print "\n\n", os.path.split(output_dir)[1]

        # clear the cache, to avoid you doing it twice
        self.state.clear()
        self._clear_cache()

    def run_mug_shelf_manipulation(self, use_debug_speed=True):
        """
        Runs the object manipulation code. Will put the object into the
        specified target pose from `run_category_manipulation_goal_estimation`
        :return:
        """

        self.wait_for_category_manipulation_goal_result()

        if not self.check_category_goal_estimation_succeeded():
            self.state.set_status("PLANNING_FAILED")
            return False

        category_manipulation_type = self.state.cache['category_manipulation_goal']['type']
        assert category_manipulation_type == CategoryManipulationType.MUG_ON_SHELF_3D

        self.moveHome()


        result = self.state.cache['category_manipulation_goal']['result']

        print("\n\n---result----\n\n", result)
        print("\n\n\n")

        T_W_fingertip = ros_numpy.numpify(result.T_world_gripper_fingertip)
        T_W_fingertip_vtk = transformUtils.getTransformFromNumpy(T_W_fingertip)

        grasp_data = GraspData.from_gripper_fingertip_frame(T_W_fingertip)
        grasp_data.gripper.params["hand_inner_diameter"] = result.gripper_width

        # rotate grasp frame to align with nominal if we are doing a vertical grasp

        force_threshold_magnitude = 30
        push_in_distance = 0.0

        if result.mug_orientation == "HORIZONTAL":
            push_in_distance = -0.005
            force_threshold_magnitude = 30
        elif result.mug_orientation == "UPRIGHT":
            push_in_distance = 0.01
            force_threshold_magnitude = 30

            # params = self.getParamsForCurrentLocation()
            # grasp_z_axis_nominal = np.array(params['grasp']['grasp_nominal_direction'])
            # grasp_data.rotate_grasp_frame_to_nominal(grasp_z_axis_nominal)


        self.state.grasp_data = grasp_data
        self.visualize_grasp(grasp_data)



        def vis_function():
            vis.updateFrame(T_W_fingertip_vtk, "gripper fingertip frame", scale=0.15, parent=self._vis_container)

            vis.updateFrame(grasp_data.grasp_frame, "grasp frame", scale=0.15, parent=self._vis_container)

        self.taskRunner.callOnThread(vis_function)

        # debugging
        print("visualizing grasp")
        self.visualize_grasp(grasp_data)


        # execute the grasp
        object_in_gripper = self.execute_grasp(self.state.grasp_data, close_gripper=True, use_cartesian_plan=True, push_in_distance=push_in_distance, force_threshold_magnitude=force_threshold_magnitude, ee_speed_m_s=0.1)


        T_W_G = self.state.cache['gripper_frame_at_grasp'] # this is set in execute_grasp

        pre_grasp_pose = self.state.cache['pre_grasp_ik_response'].joint_state.position
        pickup_speed = self.graspingParams['speed']['pickup']

        if not object_in_gripper:
            # open the gripper and back away
            self.gripperDriver.send_open_gripper_set_distance_from_current()
            self.robotService.moveToJointPosition(pre_grasp_pose,
                                                   maxJointDegreesPerSecond=
                                                  pickup_speed)
            return False

        # pickup the object
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pickup_speed)


        # move to above table pre grasp
        speed = self.graspingParams["speed"]["fast"]

        q = self._stored_poses_director["Grasping"]["above_table_pre_grasp"]
        self.robotService.moveToJointPosition(q,
                                              maxJointDegreesPerSecond=
                                              speed)



        q_approach = None
        if result.mug_orientation == "HORIZONTAL":

            q_nom = self._stored_poses_director["mug"]["horizontal_grasp_nominal"]

            q_approach_2 = self._stored_poses_director["mug"]["horizontal_grasp_approach_2"]

            self.robotService.moveToJointPosition(q_approach_2,
                                                  maxJointDegreesPerSecond=
                                                  speed)

        elif result.mug_orientation == "UPRIGHT":
            q_nom = self._stored_poses_director["mug"]["vertical_grasp_nominal"]
            q_approach_1 = self._stored_poses_director["mug"]["vertical_grasp_above_table"]

            self.robotService.moveToJointPosition(q_approach_1,
                                                  maxJointDegreesPerSecond=
                                                  speed)

        else:
            raise ValueError("unknown mug orientation: %s" %(result.mug_orientation))




        # compute some poses
        T_goal_obs = ros_numpy.numpify(result.T_goal_obs)  # 4 x 4 numpy matrix
        T_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_goal_obs)
        object_manip = ObjectManipulation(T_goal_object=T_goal_obs_vtk, T_W_G=T_W_G)
        object_manip.compute_transforms()
        T_W_Gn_vtk = object_manip.T_W_Gn # gripper to world for place pose


        T_pre_goal_obs = ros_numpy.numpify(result.T_pre_goal_obs)
        T_pre_goal_obs_vtk = transformUtils.getTransformFromNumpy(T_pre_goal_obs)
        object_manip_approach = ObjectManipulation(T_goal_object=T_pre_goal_obs_vtk, T_W_G=T_W_G)
        object_manip_approach.compute_transforms()
        T_W_Gn_approach_vtk = object_manip_approach.T_W_Gn


        # now convert these to ee poses
        T_W_ee_vtk = self.getIiwaLinkEEFrameFromGraspFrame(T_W_Gn_vtk)
        T_W_ee = transformUtils.getNumpyFromTransform(T_W_ee_vtk)

        T_W_ee_approach_vtk = self.getIiwaLinkEEFrameFromGraspFrame(T_W_Gn_approach_vtk)
        T_W_ee_approach = transformUtils.getNumpyFromTransform(T_W_ee_approach_vtk)


        # execute the place
        print("executing place on shelf")
        code = self.execute_place_new(T_W_ee, T_W_ee_approach, q_nom=q_nom, use_cartesian_plan=True, force_threshold_magnitude=30)

        return code


    def retract_from_mug_shelf(self, gripper_open=True, use_debug_speed=True):
        """
        Move backwards from the rack
        :return:
        :rtype:
        """

        category_manipulation_type = self.state.cache['category_manipulation_goal']['type']
        assert category_manipulation_type == CategoryManipulationType.MUG_ON_SHELF_3D

        result = self.state.cache['category_manipulation_goal']['result']

        if gripper_open:
            if result.mug_orientation == "HORIZONTAL":
                self.gripperDriver.sendOpenGripperCommand()
            else:
                self.gripperDriver.send_open_gripper_set_distance_from_current()

        # do different things depending on whether it was horizontal or vertical drop
        result = self.state.cache['category_manipulation_goal']['result']
        mug_orientation = result.mug_orientation

        xyz_goal = np.array([-0.10, 0, 0])
        ee_frame_id = "iiwa_link_ee"
        expressed_in_frame = ee_frame_id
        cartesian_grasp_speed = self.graspingParams['speed']['cartesian_grasp']
        cartesian_traj_goal = \
            control_utils.make_cartesian_trajectory_goal(xyz_goal,
                                                         ee_frame_id,
                                                         expressed_in_frame,
                                                         speed=cartesian_grasp_speed)

        action_client = self.robotService.cartesian_trajectory_action_client
        action_client.send_goal(cartesian_traj_goal)

        # wait for result
        action_client.wait_for_result()
        result = action_client.get_result()

        # now move to nominal position for the place
        speed = self.graspingParams["speed"]["fast"]
        super_fast_speed = q = self.graspingParams["speed"]["super_fast"]
        if use_debug_speed:
            speed = DEBUG_SPEED
        if mug_orientation == "UPRIGHT":
            q_pose_1 = self._stored_poses_director["mug"]["vertical_grasp_above_table"]

            self.robotService.moveToJointPosition(q_pose_1,
                                                  maxJointDegreesPerSecond=
                                                  super_fast_speed)

        elif mug_orientation=="HORIZONTAL":
            q_pose_1 = self._stored_poses_director["mug"]["horizontal_grasp_approach"]
            self.robotService.moveToJointPosition(q_pose_1,
                                                  maxJointDegreesPerSecond=
                                                  speed)

            q_pose_2 = self._stored_poses_director["Grasping"]["above_table_pre_grasp"]
            self.robotService.moveToJointPosition(q_pose_2,
                                                  maxJointDegreesPerSecond=
                                                  super_fast_speed)



        if EXPERIMENT_MODE:
            # move to pose
            q = self._stored_poses_director["left_table"]["look_at_mug_shelf_2"]
            self.robotService.moveToJointPosition(q, maxJointDegreesPerSecond=super_fast_speed)
            msg = self.captureRgbdAndCameraTransform()
            save_dir = os.path.join(spartanUtils.get_sandbox_dir(),
                                    self.state.cache['keypoint_detection_result']['output_dir'], "evaluation")

            self.save_RGBD_client.wait_for_server()

            goal = pdc_ros_msgs.msg.KeypointDetectionGoal()
            goal.rgbd_with_pose_list.append(msg)
            goal.camera_info = self.camera_info_subscriber.waitForNextMessage()
            goal.output_dir = save_dir

            self.save_RGBD_client.send_goal(goal)
            self.save_RGBD_client.wait_for_result()


        super_fast_speed = q = self.graspingParams["speed"]["super_fast"]
        self.moveHome(speed=super_fast_speed)


    def run_category_manipulation_pipeline(self):
        self._clear_cache()
        self.run_keypoint_detection()
        self.run_category_manipulation_goal_estimation()
        self.run_manipulate_object()


    def visualize_poser_result(self):
        """
        DEPRECATED (this code is best used from pdc_ros)
        Visualize the poser output
        """

        # debugging
        if self.poser_result is None:
            # use the default path for debugging purposes
            path_to_poser_output = os.path.join(spartanUtils.get_sandbox_dir(), "poser")
        else:
            path_to_poser_output = os.path.join(spartanUtils.get_sandbox_dir(), self.poser_result.poser_output_folder)

        self._poser_visualizer = PoserVisualizer(path_to_poser_output)
        poser_response = self._poser_visualizer.load_poser_response()
        self._poser_visualizer.visualize_result(poser_response)


    def grasp_best_match(self):
        assert self.best_match_result.match_found

        best_match_location_msg = self.best_match_result.best_match_location
        best_match_location = np.zeros(3)
        best_match_location[0] = best_match_location_msg.x
        best_match_location[1] = best_match_location_msg.y
        best_match_location[2] = best_match_location_msg.z

        # check that it is above table
        min_pt = np.array([0.4, -0.357198029757, 0.0])
        max_pt = np.array([0.822621226311, 0.3723, 0.5])

        greater_than_min = (best_match_location > min_pt).all()
        less_than_max = (best_match_location < max_pt).all()

        if not (greater_than_min and less_than_max):
            print "best match location is outside of workspace bounds"
            print "best_match_location:", best_match_location
            return False

        print "requesting Grasp 3D location"
        self.grasp_3D_location_request(best_match_location)
        result = self.wait_for_grasp_3D_location_result()
        print "received Grasp 3D Location Response"

        print "result:\n", result
        grasp_found = self.processGenerateGraspsResult(result)

        if not grasp_found:
            print "no grasp found, returning"
            return False

        print "attempting grasp"
        return self.attemptGrasp(self.graspFrame)

    def find_best_match_and_grasp_and_stow(self):

        # find best match
        result = self.findBestBatch()

        if not result.match_found:
            return False

        # attempt grasp best match
        grasp_successful = self.grasp_best_match()
        if not grasp_successful:
            self.gripperDriver.send_open_gripper_set_distance_from_current()
            self.moveHome()

            print "grasp attempt failed, resetting"
            return False

        # stow
        stow_pose = self.graspingParams["poses"]["hand_to_human_right"]
        # stow_pose = self.graspingParams["poses"]["stow_in_bin"]
        self.pickupObject(stow=True, stow_pose=stow_pose)

    def request_best_match(self):
        goal = pdc_ros_msgs.msg.FindBestMatchGoal()
        goal.rgbd_with_pose_list = self.list_rgbd_with_pose_msg
        goal.camera_info = self.camera_info_subscriber.waitForNextMessage()

        self.find_best_match_client.send_goal(goal)
        self.moveHome()

    # From: https://www.programcreek.com/python/example/99841/sensor_msgs.msg.PointCloud2

    def pointcloud2_to_array(self, cloud_msg):
        ''' 
        Converts a rospy PointCloud2 message to a numpy recordarray 
        
        Assumes all fields 32 bit floats, and there is no padding.
        '''
        dtype_list = [(f.name, np.float32) for f in cloud_msg.fields]
        cloud_arr = np.fromstring(cloud_msg.data, dtype_list)
        return cloud_arr
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

    def processGenerateGraspsResult(self, result):
        """
        Takes the result of spartan_grasp and parses it into a usable form
        :param result:
        :return:
        """
        print "num antipodal grasps = ", len(result.antipodal_grasps)
        print "num volume grasps = ", len(result.volume_grasps)

        if (len(result.antipodal_grasps) == 0) and (len(result.volume_grasps) == 0):
            self.topGrasp = None
            self._grasp_found = False
            rospy.loginfo("no valid grasps found")
            return False

        if len(result.antipodal_grasps) > 0:
            self._grasp_found = True
            grasp_msg = result.antipodal_grasps[0]
            print "top grasp was ANTIPODAL"

        elif len(result.volume_grasps) > 0:
            self._grasp_found = True
            grasp_msg = result.volume_grasps[0]
            print "top grasp was VOLUME"

        self.topGrasp = grasp_msg
        rospy.loginfo("-------- top grasp score = %.3f", self.topGrasp.score)
        self.graspFrame = spartanUtils.transformFromROSPoseMsg(self.topGrasp.pose.pose)
        self.rotateGraspFrameToAlignWithNominal(self.graspFrame)
        return True

    def make_grasp_data_from_spartan_grasp_result(self, result):
        """
        Takes the result of spartan_grasp and parses it into a usable form
        :param result:
        :return: bool, GraspData
        """
        print "num antipodal grasps = ", len(result.antipodal_grasps)
        print "num volume grasps = ", len(result.volume_grasps)

        if (len(result.antipodal_grasps) == 0) and (len(result.volume_grasps) == 0):
            rospy.loginfo("no valid grasps found")
            return False, False

        if len(result.antipodal_grasps) > 0:
            grasp_msg = result.antipodal_grasps[0]
            type = "antipodal"
            print "top grasp was ANTIPODAL"

        elif len(result.volume_grasps) > 0:
            grasp_msg = result.volume_grasps[0]
            type = "volume"
            print "top grasp was VOLUME"


        rospy.loginfo("-------- top grasp score = %.3f", grasp_msg.score)

        grasp_data = GraspData.from_spartan_grasp(grasp_msg)
        grasp_data.data['type'] = type

        # rotate the grasp to align with nominal
        params = self.getParamsForCurrentLocation()
        grasp_z_axis_nominal = np.array(params['grasp']['grasp_nominal_direction'])
        grasp_data.rotate_grasp_frame_to_nominal(grasp_z_axis_nominal)

        return True, grasp_data

    def getIiwaLinkEEFrameFromGraspFrame(self, graspFrame):
        return transformUtils.concatenateTransforms([self.iiwaLinkEEToGraspFrame, graspFrame])


    def get_iiwa_link_ee_from_gripper_fingertip_frame(self, T_W__gripper_fingertip):
        """

        :param T_gripper_fingertip__W: gripper fingertip to world transform
        :return:
        """

        return transformUtils.concatenateTransforms([self.T_gripper_fingertip__iiwa_link_ee, T_W__gripper_fingertip])

    def moveToFrame(self, graspFrame, speed=None):
        if speed is None:
            speed = self.config['grasp_speed']
        poseStamped = self.makePoseStampedFromGraspFrame(graspFrame)
        return self.robotService.moveToCartesianPosition(poseStamped, speed)

    def makePoseStampedFromGraspFrame(self, graspFrame):
        """
        Make PoseStamped message for the end effector frame from a given grasp frame
        :param graspFrame: vtkTransform of the gripper frame
        :return : pose of the end-effector for that grasp frame location
        :rtype : geometry_msgs/PoseStamped
        """
        iiwaLinkEEFrame = self.getIiwaLinkEEFrameFromGraspFrame(graspFrame)
        poseDict = spartanUtils.poseFromTransform(iiwaLinkEEFrame)
        poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"

        return poseStamped

    def make_ee_pose_stamped_from_grasp(self, T_W_gripper_fingertip):
        """
        Make PoseStamped message for the end effector frame from a given grasp frame.

        :param T_W_gripper_fingertip: The position of the tips of the fingers, move down 3 cm to get
        :return : pose of the end-effector for that grasp frame location
        :rtype : geometry_msgs/PoseStamped
        """

        iiwaLinkEEFrame = self.get_iiwa_link_ee_from_gripper_fingertip_frame(T_W_gripper_fingertip)
        poseDict = spartanUtils.poseFromTransform(iiwaLinkEEFrame)
        poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"

        return poseStamped


    def execute_grasp(self, grasp_data=None, close_gripper=True, use_cartesian_plan=True, stop_at_pre_grasp=False, push_in_distance=None, use_debug_speed=False, force_threshold_magnitude=None, ee_speed_m_s=0.05):
        """
        Moves to pre-grasp frame, then grasp frame
        attemps to close gripper if `close_gripper=True` was passed in
        :return: bool (whether or not grasp was successful)
        """

        if grasp_data is None:
            grasp_data = self.state.grasp_data

        if push_in_distance is None:
            push_in_distance = self.graspingParams['grasp_push_in_distance']

        gripper_width = grasp_data.grasp_inner_diameter
        if gripper_width is not None:
            gripper_driver_width = gripper_width + self.graspingParams['gripper_width_offset']
            self.gripperDriver.sendGripperCommand(gripper_driver_width, force=20.0)
        else:
            self.gripperDriver.send_open_gripper_set_distance_from_current()

        rospy.sleep(0.5)  # wait for 0.5 for gripper to move



        # compute the pre-grasp frame
        pre_grasp_distance = self.graspingParams['pre_grasp_distance']
        pre_grasp_frame_gripper = grasp_data.compute_pre_grasp_frame(distance=pre_grasp_distance)

        pre_grasp_ee_pose_stamped = self.makePoseStampedFromGraspFrame(pre_grasp_frame_gripper)

        # safety check
        is_safe = (GraspData.grasp_frame_safety_check(grasp_data.grasp_frame) and GraspData.grasp_frame_safety_check(pre_grasp_frame_gripper))
        if not is_safe:
            self.state.set_status("SAFETY_CHECK_FAILED")
            return False


        # run the ik for moving to pre-grasp location
        graspLocationData = self.graspingParams[self.state.graspingLocation]
        above_table_pre_grasp = graspLocationData['poses']['above_table_pre_grasp']
        pre_grasp_ik_response = self.robotService.runIK(pre_grasp_ee_pose_stamped,
                                                        seedPose=above_table_pre_grasp,
                                                        nominalPose=above_table_pre_grasp)

        pre_grasp_pose = pre_grasp_ik_response.joint_state.position

        if not pre_grasp_ik_response.success:
            rospy.loginfo("pre grasp pose ik failed, returning")
            self.state.set_status_ik_failed()
            self.state.print_status()
            return False

        # run the ik for moving to grasp location
        # for now just do IK, otherwise use cartesian space plan with force guards
        grasp_frame_ee_pose_stamped = self.makePoseStampedFromGraspFrame(grasp_data.grasp_frame)
        grasp_ik_response = self.robotService.runIK(grasp_frame_ee_pose_stamped,
                                                    seedPose=above_table_pre_grasp,
                                                    nominalPose=above_table_pre_grasp)

        grasp_pose = grasp_ik_response.joint_state.position
        if not grasp_ik_response.success:
            rospy.loginfo("pre grasp pose ik failed, returning")
            self.state.set_status_ik_failed()
            self.state.print_status()
            return False

        # store for later use
        self.state.cache['grasp_ik_response'] = grasp_ik_response
        self.state.cache['pre_grasp_ik_response'] = pre_grasp_ik_response

        # move to pre-grasp position
        # we do this using a position trajectory
        print "moving to pre-grasp"
        pre_grasp_speed = self.graspingParams['speed']['pre_grasp']

        #### debugging
        speed = pre_grasp_speed
        if use_debug_speed:
            speed = DEBUG_SPEED
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              speed)

        self.state.set_status("PRE_GRASP")
        print "at pre-grasp pose"

        if stop_at_pre_grasp:
            return

        if use_cartesian_plan:
            # move to grasp position using compliant cartesian plan

            move_forward_distance = pre_grasp_distance + push_in_distance
            print "move_forward_distance", move_forward_distance
            xyz_goal = move_forward_distance * np.array([1, 0, 0])
            ee_frame_id = "iiwa_link_ee"
            expressed_in_frame = ee_frame_id
            cartesian_grasp_speed = self.graspingParams['speed']['cartesian_grasp']
            cartesian_grasp_speed = ee_speed_m_s
            cartesian_traj_goal = \
                control_utils.make_cartesian_trajectory_goal(xyz_goal,
                                                             ee_frame_id,
                                                             expressed_in_frame,
                                                             speed=cartesian_grasp_speed)

            # add force guards
            # -z (gripper) direction in frame iiwa_link_ee,
            if force_threshold_magnitude is None:
                force_threshold_magnitude = self.graspingParams['force_threshold_magnitude']
            force_vector = force_threshold_magnitude * np.array([-1, 0, 0])
            force_guard = control_utils.make_force_guard_msg(force_vector)

            cartesian_traj_goal.force_guard.append(force_guard)
            action_client = self.robotService.cartesian_trajectory_action_client
            action_client.send_goal(cartesian_traj_goal)

            # wait for result
            action_client.wait_for_result()
            result = action_client.get_result()
            grasp_data.data['cartesian_trajectory_result'] = result

            print "Cartesian Trajectory Result\n", result
        else:
            # move to grasp pose using standard IK
            speed = self.graspingParams['speed']['grasp']
            if use_debug_speed:
                speed = DEBUG_SPEED
            self.robotService.moveToJointPosition(grasp_pose,
                                                  maxJointDegreesPerSecond=
                                                  speed)


        # record current location of gripper (in world frame)
        # before closing the gripper

        pos, quat = self.get_transform("iiwa_link_ee", "base")
        T_world_ee = transformUtils.transformFromPose(pos, quat)
        T_world_grasp = transformUtils.concatenateTransforms([self.graspToIiwaLinkEE, T_world_ee])
        self.state.cache['gripper_frame_at_grasp'] = T_world_grasp

        has_object = False
        if close_gripper:
            print "closing gripper"
            has_object = self.gripperDriver.closeGripper()

            if has_object:
                self.state.set_status("OBJECT_IN_GRIPPER")
                print "object in gripper"
            else:
                self.state.set_status("GRASP_FAILED")
                print "grasp failed"



        return has_object


    def execute_place(self, grasp_data=None, use_cartesian_plan=True):

        if grasp_data is None:
            grasp_data = self.state.grasp_data

        # compute the pre-grasp frame
        pre_grasp_distance = self.graspingParams['pre_grasp_distance']
        pre_grasp_frame_gripper = grasp_data.compute_pre_grasp_frame(distance=pre_grasp_distance)

        pre_grasp_ee_pose_stamped = self.makePoseStampedFromGraspFrame(pre_grasp_frame_gripper)

        # run the ik for moving to pre-grasp location
        graspLocationData = self.graspingParams[self.state.graspingLocation]
        above_table_pre_grasp = graspLocationData['poses']['above_table_pre_grasp']
        pre_grasp_ik_response = self.robotService.runIK(pre_grasp_ee_pose_stamped,
                                                        seedPose=above_table_pre_grasp,
                                                        nominalPose=above_table_pre_grasp)

        pre_grasp_pose = pre_grasp_ik_response.joint_state.position

        if not pre_grasp_ik_response.success:
            rospy.loginfo("pre grasp pose ik failed, returning")
            self.state.set_status_ik_failed()
            self.state.print_status()
            return False

        # run the ik for moving to grasp location
        # for now just do IK, otherwise use cartesian space plan with force guards
        grasp_frame_ee_pose_stamped = self.makePoseStampedFromGraspFrame(grasp_data.grasp_frame)
        grasp_ik_response = self.robotService.runIK(grasp_frame_ee_pose_stamped,
                                                    seedPose=above_table_pre_grasp,
                                                    nominalPose=above_table_pre_grasp)

        grasp_pose = grasp_ik_response.joint_state.position
        if not grasp_ik_response.success:
            rospy.loginfo("pre grasp pose ik failed, returning")
            self.state.set_status_ik_failed()
            self.state.print_status()
            return False

        # store for later use
        self.state.cache['grasp_ik_response'] = grasp_ik_response
        self.state.cache['pre_grasp_ik_response'] = pre_grasp_ik_response

        # move to pre-grasp position
        # we do this using a position trajectory
        print "moving to pre-grasp"
        pre_grasp_speed = self.graspingParams['speed']['pre_grasp']
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pre_grasp_speed)

        self.state.set_status("PRE_GRASP")
        print "at pre-grasp pose"

        if use_cartesian_plan:
            # move to grasp position using compliant cartesian plan

            push_distance = self.graspingParams['grasp_push_in_distance']
            move_forward_distance = pre_grasp_distance + push_distance
            print "move_forward_distance", move_forward_distance
            xyz_goal = move_forward_distance * np.array([1, 0, 0])
            ee_frame_id = "iiwa_link_ee"
            expressed_in_frame = ee_frame_id
            cartesian_grasp_speed = self.graspingParams['speed']['cartesian_grasp']
            cartesian_traj_goal = \
                control_utils.make_cartesian_trajectory_goal(xyz_goal,
                                                             ee_frame_id,
                                                             expressed_in_frame,
                                                             speed=cartesian_grasp_speed)

            # add force guards
            # -z (gripper) direction in frame iiwa_link_ee,
            force_magnitude = self.graspingParams['force_threshold_magnitude']
            force_vector = force_magnitude * np.array([-1, 0, 0])
            force_guard = control_utils.make_force_guard_msg(force_vector)

            cartesian_traj_goal.force_guard.append(force_guard)
            action_client = self.robotService.cartesian_trajectory_action_client
            action_client.send_goal(cartesian_traj_goal)

            # wait for result
            action_client.wait_for_result()
            result = action_client.get_result()
            grasp_data.data['cartesian_trajectory_result'] = result

            print "Cartesian Trajectory Result\n", result
        else:
            # move to grasp pose using standard IK
            speed = self.graspingParams['speed']['grasp']
            self.robotService.moveToJointPosition(grasp_pose,
                                                  maxJointDegreesPerSecond=
                                                  speed)

        self.gripperDriver.send_open_gripper_set_distance_from_current()
        return True

    def execute_place_new(self, T_W_ee, T_W_ee_approach, q_nom=None, use_cartesian_plan=False, use_debug_speed=False, force_threshold_magnitude=10, ee_speed_m_s=0.05):
        """

        :param T_W_ee: ee location for place
        :type T_W_ee:
        :param T_W_ee_approach: ee location for approach
        :type T_W_ee_approach:
        :param q_nom: pose for use as nominal and seed for ik
        :type q_nom:
        :param use_cartesian_plan: whether or not to use the cartesian plane
        :type use_cartesian_plan:
        :return:
        :rtype:
        """

        # safety check
        is_safe = (GraspData.grasp_frame_safety_check(T_W_ee) and GraspData.grasp_frame_safety_check(T_W_ee_approach))
        if not is_safe:
            self.state.set_status("SAFETY_CHECK_FAILED")
            return False

        # run the ik for moving to pre-grasp location
        debug_speed = 10

        if q_nom is None:
            graspLocationData = self.graspingParams[self.state.graspingLocation]
            q_nom = graspLocationData['poses']['above_table_pre_grasp']

        T_W_ee_vtk = transformUtils.getTransformFromNumpy(T_W_ee)
        T_W_ee_approach_vtk = transformUtils.getTransformFromNumpy(T_W_ee_approach)

        # pose stamped
        frame_id = "base"
        T_W_ee_approach_stamped = geometry_msgs.msg.PoseStamped()
        T_W_ee_approach_stamped.pose = ros_numpy.msgify(geometry_msgs.msg.Pose,
                                               T_W_ee_approach)
        T_W_ee_approach_stamped.header.frame_id = frame_id
        T_W_ee_approach_stamped.header.stamp = rospy.Time.now()

        print T_W_ee_approach_stamped

        pre_place_ik_response = self.robotService.runIK(T_W_ee_approach_stamped,
                                                        seedPose=q_nom,
                                                        nominalPose=q_nom)

        pre_place_pose = pre_place_ik_response.joint_state.position

        self.state.cache["pre_place_ik_response"] = pre_place_ik_response

        if not pre_place_ik_response.success:
            rospy.loginfo("pre place pose ik failed, returning")
            self.state.set_status_ik_failed()
            self.state.print_status()
            return False

        # run the ik for moving to grasp location
        frame_id = "base"
        T_W_ee_stamped = geometry_msgs.msg.PoseStamped()
        T_W_ee_stamped.pose = ros_numpy.msgify(geometry_msgs.msg.Pose,
                                                        T_W_ee)
        T_W_ee_stamped.header.frame_id = frame_id
        T_W_ee_stamped.header.stamp = rospy.Time.now()


        # for now just do IK, otherwise use cartesian space plan with force guards
        place_ik_response = self.robotService.runIK(T_W_ee_stamped,
                                                        seedPose=q_nom,
                                                        nominalPose=q_nom)

        place_pose = place_ik_response.joint_state.position
        if not place_ik_response.success:
            rospy.loginfo("place pose ik failed, returning")
            self.state.set_status_ik_failed()
            self.state.print_status()
            return False

        # store for later use
        self.state.cache['place_ik_response'] = place_ik_response

        # move to pre-grasp position
        # we do this using a position trajectory
        print "moving to approach pose"
        # pre_grasp_speed = self.graspingParams['speed']['pre_grasp']
        speed = self.graspingParams['speed']['grasp']
        if use_debug_speed:
            speed = debug_speed
        self.robotService.moveToJointPosition(pre_place_pose,
                                              maxJointDegreesPerSecond=
                                              speed)

        self.state.set_status("PRE_GRASP")
        print "at approach pose"

        if use_cartesian_plan:
            # move to grasp position using compliant cartesian plan
            # for now doesn't deal with orientations

            xyz_approach = np.array(T_W_ee_approach_vtk.GetPosition())
            xyz_place = np.array(T_W_ee_vtk.GetPosition())
            distance = np.linalg.norm(xyz_place - xyz_approach)
            duration = distance/ee_speed_m_s


            xyz_goal = xyz_place
            ee_frame_id = "iiwa_link_ee"
            base_frame_id = "base"
            expressed_in_frame = base_frame_id
            cartesian_grasp_speed = self.graspingParams['speed']['cartesian_grasp']
            cartesian_traj_goal = \
                control_utils.make_cartesian_trajectory_goal(xyz_goal,
                                                             ee_frame_id,
                                                             expressed_in_frame,
                                                             duration=duration)


            # add force guards
            # x_axis in frame iiwa_link_ee,
            force_vector = force_threshold_magnitude * np.array([-1, 0, 0])
            force_guard = control_utils.make_force_guard_msg(force_vector)
            cartesian_traj_goal.force_guard.append(force_guard)

            # z_axis in frame iiwa_link_ee
            force_vector = force_threshold_magnitude * np.array([0, 0, 1])
            force_guard = control_utils.make_force_guard_msg(force_vector)
            cartesian_traj_goal.force_guard.append(force_guard)


            action_client = self.robotService.cartesian_trajectory_action_client
            action_client.send_goal(cartesian_traj_goal)

            # wait for result
            action_client.wait_for_result()
            result = action_client.get_result()
            self.state.cache['cartesian_traj_result'] = result

            print "Cartesian Trajectory Result\n", result
        else:
            # move to grasp pose using standard IK
            speed = self.graspingParams['speed']['grasp']
            self.robotService.moveToJointPosition(place_pose,
                                                  maxJointDegreesPerSecond=
                                                  speed)

        # now back off

        # self.gripperDriver.send_open_gripper_set_distance_from_current()
        return True

    def attemptGrasp(self, graspFrame):
        """
        Attempt a grasp
        return: boolean if it was successful or not
        """

        self._clear_cache()
        self._cache["grasp_frame"] = graspFrame

        preGraspFrame = transformUtils.concatenateTransforms([self.preGraspToGraspTransform, self.graspFrame])

        graspLocationData = self.graspingParams[self.state.graspingLocation]
        above_table_pre_grasp = graspLocationData['poses']['above_table_pre_grasp']
        preGraspFramePoseStamped = self.makePoseStampedFromGraspFrame(preGraspFrame)
        preGrasp_ik_response = self.robotService.runIK(preGraspFramePoseStamped, seedPose=above_table_pre_grasp,
                                                       nominalPose=above_table_pre_grasp)

        if not preGrasp_ik_response.success:
            rospy.loginfo("pre grasp pose ik failed, returning")
            return False

        graspFramePoseStamped = self.makePoseStampedFromGraspFrame(graspFrame)
        preGraspPose = preGrasp_ik_response.joint_state.position

        grasp_ik_response = self.robotService.runIK(graspFramePoseStamped, seedPose=preGraspPose,
                                                    nominalPose=preGraspPose)

        self._cache['grasp_ik_response'] = grasp_ik_response
        self._cache['pre_grasp_ik_response'] = preGrasp_ik_response

        if not grasp_ik_response.success:
            rospy.loginfo("grasp pose not reachable, returning")
            return False

        graspPose = grasp_ik_response.joint_state.position

        # store for future use
        self.preGraspFrame = preGraspFrame
        self.graspFrame = graspFrame

        self.gripperDriver.send_open_gripper_set_distance_from_current()
        rospy.sleep(0.5)  # wait for the gripper to open
        self.robotService.moveToJointPosition(preGraspPose,
                                              maxJointDegreesPerSecond=self.graspingParams['speed']['pre_grasp'])
        self.robotService.moveToJointPosition(graspPose, maxJointDegreesPerSecond=self.graspingParams['speed']['grasp'])

        objectInGripper = self.gripperDriver.closeGripper()
        return objectInGripper

    def vtkFrameToPoseMsg(self, vtkFrame):
        poseDict = spartanUtils.poseFromTransform(vtkFrame)
        poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"

        return poseStamped

    """
    Moves the gripper up 15cm then moves home
    """

    def pickupObject(self, stow=True, stow_pose=None):

        endEffectorFrame = self.tfBuffer.lookup_transform(self.config['base_frame_id'],
                                                          self.config['end_effector_frame_id'], rospy.Time(0))

        eeFrameVtk = spartanUtils.transformFromROSTransformMsg(endEffectorFrame.transform)
        eeFrameVtk.PostMultiply()
        eeFrameVtk.Translate(0, 0, self.config['pick_up_distance'])

        vis.updateFrame(eeFrameVtk, 'pickup frame')

        self._cache['eeFrameVtk'] = eeFrameVtk
        self._cache['endEffectorFrame'] = endEffectorFrame

        poseStamped = self.vtkFrameToPoseMsg(eeFrameVtk)
        speed = 10  # joint degrees per second
        params = self.getParamsForCurrentLocation()
        above_table_pre_grasp = params['poses']['above_table_pre_grasp']
        ik_response = self.robotService.runIK(poseStamped, seedPose=above_table_pre_grasp,
                                              nominalPose=above_table_pre_grasp)

        if ik_response.success:
            self.robotService.moveToJointPosition(ik_response.joint_state.position,
                                                  maxJointDegreesPerSecond=self.graspingParams['speed']['slow'])

        if stow_pose is None:
            stow_pose = self.getStowPose()

        # move to above_table_pre_grasp
        # self.robotService.moveToJointPosition(above_table_pre_grasp, maxJointDegreesPerSecond=self.graspingParams['speed']['stow'])

        # move to stow_pose
        if stow:
            self.robotService.moveToJointPosition(stow_pose,
                                                  maxJointDegreesPerSecond=self.graspingParams['speed']['stow'])

        # release object
        self.gripperDriver.send_open_gripper_set_distance_from_current()
        rospy.sleep(0.5)

        # move Home
        self.moveHome()

    def pickup_object(self):
        """
        Just moves to pre-grasp frame
        :return:
        """

        if "pre_grasp_ik_response" not in self.state.cache:
            return False

        pre_grasp_ik_response = self.state.cache['pre_grasp_ik_response']
        pre_grasp_pose = pre_grasp_ik_response.joint_state.position
        pre_grasp_speed = self.graspingParams['speed']['stow']
        self.robotService.moveToJointPosition(pre_grasp_pose,
                                              maxJointDegreesPerSecond=
                                              pre_grasp_speed)


    def pickup_object_and_reorient_on_table(self):
        """
        Places the object back on the table in a random orientation
        Relies on variables in self._cache being set from when we picked up the object
        :return:
        """

        def set_position(t, pos):
            _, quat = transformUtils.poseFromTransform(t)
            return transformUtils.transformFromPose(pos, quat)

        speed = self.config["object_interaction"]["speed"]
        pick_up_distance = self.config["object_interaction"]["pickup_distance"]
        drop_distance_above_grasp = self.config["object_interaction"]["drop_distance_above_grasp"]
        rotate_speed = self.config["object_interaction"]["rotate_speed"]
        drop_location = self.config["object_interaction"]["drop_location"]  # z coordinate is overwritten later

        endEffectorFrame = self.tfBuffer.lookup_transform(self.config['base_frame_id'],
                                                          self.config['end_effector_frame_id'], rospy.Time(0))

        grasp_ee_frame = spartanUtils.transformFromROSTransformMsg(endEffectorFrame.transform)

        # the frame of the end-effector after we have picked up the object
        pickup_ee_frame_vtk = transformUtils.copyFrame(grasp_ee_frame)
        pickup_ee_frame_vtk.PostMultiply()
        pickup_ee_frame_vtk.Translate(0, 0, pick_up_distance)

        vis.updateFrame(pickup_ee_frame_vtk, 'pickup frame', scale=0.15)

        self._cache['grasped_ee_frame'] = endEffectorFrame
        self._cache['pickup_ee_frame_vtk'] = pickup_ee_frame_vtk

        poseStamped = self.vtkFrameToPoseMsg(pickup_ee_frame_vtk)
        speed = 10  # joint degrees per second
        params = self.getParamsForCurrentLocation()
        above_table_pre_grasp = params['poses']['above_table_pre_grasp']
        pickup_ik_response = self.robotService.runIK(poseStamped, seedPose=above_table_pre_grasp,
                                                     nominalPose=above_table_pre_grasp)

        # compute the drop frame location
        # This is done by rotating along the z-axis of the grasp frame by some random
        # amount in [-90, 90] and then just releasing


        rotate_x_angle = random.uniform(45, 90)
        # if random.random() < 0.5:
        #     rotate_x_angle *= -1



        pre_drop_frame = transformUtils.copyFrame(pickup_ee_frame_vtk)
        pre_drop_frame.PreMultiply()
        pre_drop_frame.RotateX(rotate_x_angle)
        pre_drop_frame_pos, _ = transformUtils.poseFromTransform(pre_drop_frame)
        pre_drop_frame_pos[0:2] = drop_location[0:2]
        pre_drop_frame = set_position(pre_drop_frame, pre_drop_frame_pos)

        grasp_ee_height = grasp_ee_frame.GetPosition()[2]
        drop_frame_pos = copy.copy(pre_drop_frame_pos)
        drop_frame_pos[2] = grasp_ee_height + drop_distance_above_grasp

        print "drop_frame_pos", drop_frame_pos

        drop_frame = transformUtils.copyFrame(pre_drop_frame)
        drop_frame = set_position(drop_frame, drop_frame_pos)

        vis.updateFrame(pre_drop_frame, "pre drop frame", scale=0.15)
        vis.updateFrame(drop_frame, "drop frame", scale=0.15)

        # run IK
        pre_drop_frame_pose_stamped = self.vtkFrameToPoseMsg(pre_drop_frame)
        pre_drop_ik_response = self.robotService.runIK(pre_drop_frame_pose_stamped, seedPose=above_table_pre_grasp,
                                                       nominalPose=above_table_pre_grasp)

        drop_frame_pose_stamped = self.vtkFrameToPoseMsg(drop_frame)
        drop_ik_response = self.robotService.runIK(drop_frame_pose_stamped, seedPose=above_table_pre_grasp,
                                                   nominalPose=above_table_pre_grasp)

        if pickup_ik_response.success and pre_drop_ik_response.success and drop_ik_response.success:
            # pickup object
            self.robotService.moveToJointPosition(pickup_ik_response.joint_state.position,
                                                  maxJointDegreesPerSecond=speed)

            # move to pre-drop
            self.robotService.moveToJointPosition(pre_drop_ik_response.joint_state.position,
                                                  maxJointDegreesPerSecond=rotate_speed)

            # move to drop location
            self.robotService.moveToJointPosition(drop_ik_response.joint_state.position,
                                                  maxJointDegreesPerSecond=speed)

            self.gripperDriver.send_open_gripper_set_distance_from_current()
            rospy.sleep(0.5)

            # move to pre-drop
            self.robotService.moveToJointPosition(pre_drop_ik_response.joint_state.position,
                                                  maxJointDegreesPerSecond=rotate_speed)

            self.moveHome()

        else:
            print "ik failed"
            return False

        return True

    def planGraspAndPickupObject(self, stow=True):
        self.collectSensorData()
        self.requestGrasp()
        self.moveHome()
        result = self.waitForGenerateGraspsResult()
        graspFound = self.processGenerateGraspsResult(result)

        if not graspFound:
            rospy.loginfo("no grasp found, returning")
            return False

        graspSuccessful = self.attemptGrasp(self.graspFrame)
        if not graspSuccessful:
            rospy.loginfo("grasp not successful returning")
            return False

        self.pickupObject(stow)

    def graspAndStowObject(self):
        graspSuccessful = self.attemptGrasp(self.graspFrame)
        if not graspSuccessful:
            rospy.loginfo("grasp not successful returning")
            return False

        stow = True
        self.pickupObject(stow)

    def askForCaptureScene(self):
        """
        This function just waits for, then asks for the capture_scene service
        provided by fusion_server.

        This only collects fusion data without performing fusion, so it's
        fast.  See fusion_server for documentation.
        """
        rospy.wait_for_service('capture_scene')
        print "Found it!, starting capture..."
        try:
            capture_scene = rospy.ServiceProxy('capture_scene', fusion_server.srv.CaptureScene)
            resp = capture_scene()
            print "bag_filepath = %s" % resp.bag_filepath
            rospy.loginfo("bag_filepath = %s", resp.bag_filepath)
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def interact_with_object(self):
        """
        Runs one iteration of picking up the object re-orienting it
        and then placing it back on the table
        """
        self.collectSensorData()
        self.moveHome()
        self.requestGrasp()
        result = self.waitForGenerateGraspsResult()
        graspFound = self.processGenerateGraspsResult(result)

        if not graspFound:
            print "no grasp found"
            return False

        grasp_successful = self.attemptGrasp(self.graspFrame)
        if not grasp_successful:
            print "grasp attemp was not successful"
            return False
        else:
            print "grasped object"

        reoriented_object = self.pickup_object_and_reorient_on_table()
        if not reoriented_object:
            print "didn't manage to reorient object"
            return False

        return True

    def interactAndCollectFusionDataLoop(self, num_interactions):
        """
        Attempts to pickup the object and move it around

        :param num_interactions:
        :return:
        """

        for i in range(num_interactions):

            success = self.interact_with_object()

            if not success:
                print "Human, please go move the object? \n"
                print "If you don't want to keep doing this,"
                print "then go implement a 'smack-the-object' primitive."
                # in future:
                # self.smackObject()
                rospy.sleep(4.0)

            rospy.sleep(1.0)
            self.askForCaptureScene()

    def testMoveToFrame(self):
        pos = [0.51148583, 0.0152224, 0.50182436]
        quat = [0.68751512, 0.15384615, 0.69882778, -0.12366916]
        targetFrame = transformUtils.transformFromPose(pos, quat)
        poseDict = spartanUtils.poseFromTransform(targetFrame)
        poseMsg = rosUtils.ROSPoseMsgFromPose(poseDict)

        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.pose = poseMsg
        poseStamped.header.frame_id = "base"
        self.poseStamped = poseStamped

        self.robotService.moveToCartesianPosition(poseStamped, 30)

    def showGraspFrame(self):
        vis.updateFrame(self.graspFrame, 'grasp frame', scale=0.15)
        vis.updateFrame(self.getIiwaLinkEEFrameFromGraspFrame(self.graspFrame), 'iiwa_link_ee_grasp_frame', scale=0.15)

    def showGripperFrame(self):
        iiwaLinkEE = self.robotSystem.robotStateModel.getLinkFrame('iiwa_link_ee')
        gripperFrame = transformUtils.concatenateTransforms([self.graspToIiwaLinkEE, iiwaLinkEE])
        vis.updateFrame(gripperFrame, 'Gripper Frame', scale=0.15)

    def show_gripper_fingertip_frame(self):
        iiwaLinkEE = self.robotSystem.robotStateModel.getLinkFrame('iiwa_link_ee')
        gripperFrame = transformUtils.concatenateTransforms([self.gripper_fingertip_to_iiwa_link_ee, iiwaLinkEE])
        vis.updateFrame(gripperFrame, 'Gripper Fingertip Frame', scale=0.15)


    def getParamsForCurrentLocation(self):
        return self.graspingParams[self.state.graspingLocation]


    def rotateGraspFrameToAlignWithNominal(self, graspFrame):
        """
        Rotate the grasp frame to align with the nominal direction. In this case we want
        the ZAxis of the grasp to be aligned with (1,0,0) in world frame.
        If it's not aligned rotate it by 180 degrees about the x-axis of the grasp
        :param graspFrame:
        :return:
        """
        graspFrameZAxis = graspFrame.TransformVector(0, 0, 1)
        params = self.getParamsForCurrentLocation()
        graspNominalDirection = params['grasp']['grasp_nominal_direction']
        if (np.dot(graspFrameZAxis, graspNominalDirection) < 0):
            graspFrame.PreMultiply()
            graspFrame.RotateX(180)

    def saveSensorDataToBagFile(self, pointCloudListMsg=None, filename=None, overwrite=True):
        """
        Save sensor data to a bag file

        """
        if pointCloudListMsg is None:
            return

        if filename is None:
            filename = os.path.join(spartanUtils.get_sandbox_dir(), "rosbag", 'grasp_sensor_data_%s.bag' %(spartanUtils.get_current_time_unique_name()))

        if not os.path.isdir(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))

        if overwrite and os.path.isfile(filename):
            os.remove(filename)

        bag = rosbag.Bag(filename, 'w')
        bag.write('data', pointCloudListMsg)
        bag.close()

    def requestGrasp(self, pointCloudListMsg=None):
        """
        Requests a grasp from the SpartanGrasp ROS service
        Doesn't collect new sensor data
        """
        # request the grasp via a ROS Action

        if pointCloudListMsg is None:
            pointCloudListMsg = self.pointCloudListMsg

        rospy.loginfo("waiting for spartan grasp server")
        self.generate_grasps_client.wait_for_server()
        rospy.loginfo("requsting grasps spartan grasp server")

        params = self.getParamsForCurrentLocation()
        goal = spartan_grasp_msgs.msg.GenerateGraspsFromPointCloudListGoal()
        

        goal.point_clouds = self.pointCloudListMsg

        if 'grasp_volume' in params:
            node = params['grasp_volume']
            rectangle = GraspSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.grasp_volume.append(rectangle)

        if 'collision_volume' in params:
            node = params['collision_volume']
            rectangle = GraspSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.collision_volume.append(rectangle)

        if 'collision_objects' in params:
            for key, val in params['collision_objects'].iteritems():
                rectangle = GraspSupervisor.rectangleMessageFromYamlNode(val)
                goal.params.collision_objects.append(rectangle)

        self.generate_grasps_client.send_goal(goal)

    def call_spartan_grasp(self):
        """
        Better named wrapper method
        :return:
        """

        self.requestGrasp()

    def waitForGenerateGraspsResult(self):
        rospy.loginfo("waiting for result")
        self.generate_grasps_client.wait_for_result()
        result = self.generate_grasps_client.get_result()
        self.generate_grasps_result = result
        rospy.loginfo("received result")

        return result

    def wait_for_grasp_3D_location_result(self):
        """
        Waits for the result of the Grasp3DLocation action
        :return:
        """
        rospy.loginfo("waiting for result")
        self.grasp_3D_location_client.wait_for_result()
        result = self.grasp_3D_location_client.get_result()
        self.grasp_3D_location_result = result  # debugging
        rospy.loginfo("received result")

        return result

    def request_grasp_3D_location(self, pointCloudListMsg=None, grasp_point=None):
        """
        Requests a grasp3DLocation from the SpartanGrasp ROS service
        Doesn't collect new sensor data
        """
        # request the grasp via a ROS Action
        if pointCloudListMsg is None:
            pointCloudListMsg = self.pointCloudListMsg

        rospy.loginfo("waiting for spartan grasp server")
        self.grasp_3D_location_client.wait_for_server()
        rospy.loginfo("requsting grasps spartan grasp server")

        params = self.getParamsForCurrentLocation()
        goal = spartan_grasp_msgs.msg.Grasp3DLocationGoal()
        if grasp_point is None:
            grasp_point = self.get_clicked_point()

        goal.grasp_point = self.get_clicked_point()
        goal.point_clouds = pointCloudListMsg


        if 'grasp_volume' in params:
            node = params['grasp_volume']
            rectangle = GraspSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.grasp_volume.append(rectangle)

        if 'collision_volume' in params:
            node = params['collision_volume']
            rectangle = GraspSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.collision_volume.append(rectangle)

        if 'collision_objects' in params:
            for key, val in params['collision_objects'].iteritems():
                rectangle = GraspSupervisor.rectangleMessageFromYamlNode(val)
                goal.params.collision_objects.append(rectangle)

        self.grasp_3D_location_client.send_goal(goal)

    def request_spartan_grasp(self, clear_state=True):
        """
        - collect sensor data
        - send request to spartan grasp
        :return: bool, GraspData
        """
        self.moveHome()
        self.collectSensorData()
        self.moveHome()
        self.requestGrasp()
        result = self.waitForGenerateGraspsResult()
        grasp_found, grasp_data = self.make_grasp_data_from_spartan_grasp_result(result)

        if clear_state:
            self.state.clear()

        if grasp_found:
            self.state.set_status("GRASP_FOUND")
            self.state.grasp_data = grasp_data
        else:
            self.state.set_status("NO_GRASP_FOUND")

        if grasp_found and self.debugMode:
            # visualize the grasp frame
            self.visualize_grasp(grasp_data)

        return grasp_found, grasp_data

    def grasp_3D_location_request(self, grasp_point, pointCloudListMsg=None):
        """
        Sends a request to grasp a specific 3D location
        :param : grasp_point is numpy array or list of size [3]
        """
        params = self.getParamsForCurrentLocation()
        goal = spartan_grasp_msgs.msg.Grasp3DLocationGoal()

        if pointCloudListMsg is None:
            goal.point_clouds = self.pointCloudListMsg

        goal.grasp_point.x = grasp_point[0]
        goal.grasp_point.y = grasp_point[1]
        goal.grasp_point.z = grasp_point[2]

        if 'grasp_volume' in params:
            node = params['grasp_volume']
            rectangle = GraspSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.grasp_volume.append(rectangle)

        if 'collision_volume' in params:
            node = params['collision_volume']
            rectangle = GraspSupervisor.rectangleMessageFromYamlNode(node)
            goal.params.collision_volume.append(rectangle)

        if 'collision_objects' in params:
            for key, val in params['collision_objects'].iteritems():
                rectangle = GraspSupervisor.rectangleMessageFromYamlNode(val)
                goal.params.collision_objects.append(rectangle)

        self.grasp_3D_location_client.send_goal(goal)

    def grasp_3D_location(self):
        """
        Runs the grasping_3D_location pipeline
        1. Checks to make sure there is a clicked_point
        2. Collects sensor data
        3. Sends off the request to spartan_grasp server
        :return: None
        """

        self.get_clicked_point()
        self.collectSensorData()
        self.request_grasp_3D_location()
        self.moveHome()
        result = self.wait_for_grasp_3D_location_result()
        grasp_found = self.processGenerateGraspsResult(result)

    def visualize_grasp(self, grasp_data):
        stamp = rospy.Time.now()
        vis.updateFrame(grasp_data.grasp_frame, "grasp frame", parent=self._vis_container,
                        scale=0.15)

        point_cloud_msg = None
        if 'point_cloud_msg' in grasp_data.data:
            point_cloud_msg = grasp_data.data['point_cloud_msg']

        # publish grasp to world transform
        pose = director_utils.poseFromTransform(grasp_data.grasp_frame)
        transform_msg = rosUtils.ROSTransformMsgFromPose(pose)

        ts = geometry_msgs.msg.TransformStamped()
        ts.header.stamp = stamp
        ts.header.frame_id = self.config["base_frame_id"]
        frame_id = "grasp_frame"
        ts.child_frame_id = frame_id
        ts.transform = transform_msg

        # use the gripper stored in the grasp data if it exists
        gripper = grasp_data.gripper
        if gripper is None:
            gripper = self._gripper

        marker_array = gripper.make_rviz_visualization_msg(frame_id, stamp)

        for i in xrange(0, 5):
            if point_cloud_msg is not None:
                self.grasp_pointcloud_publisher.publish(point_cloud_msg)

            self.rviz_marker_array_publisher.publish(marker_array)
            self.tfBroadcaster.sendTransform(ts)
            rospy.sleep(0.02)

    def get_ggcnn_grasp(self):
        """
        Looks up the ggcnn grasp frame from the tf server

        Also need to think about gripper width etc.

        :return: tuple (bool, dict)
        :rtype:
        """
        # just do a transform lookup
        return_data = dict()
        self.state.clear()
        try:
            ggcnn_grasp_frame_camera_axes = self.tfBuffer.lookup_transform(self.config["base_frame_id"],
                                                                           self.ggcnn_grasp_frame_camera_axes_id,
                                                                           rospy.Time.now(), rospy.Duration(2.0))
        except Exception as e:
            rospy.loginfo("Unable to get ggcnn grasp frame from tf, returning")
            print(e)
            return False, return_data

        return_data['ggcnn_grasp_frame_camera_axes'] = ggcnn_grasp_frame_camera_axes

        # make grasp object
        T_W_GC = director_utils.transformFromROSTransformMsg(ggcnn_grasp_frame_camera_axes.transform)
        grasp_data = GraspData.from_ggcnn_grasp_frame_camera_axes(T_W_GC)

        # get the pointcloud associated with this grasp
        point_cloud_msg = self.pointCloudSubscriber.waitForNextMessage()
        grasp_data.data['point_cloud_msg'] = point_cloud_msg

        # rotate the grasp to align with nominal
        params = self.getParamsForCurrentLocation()
        grasp_z_axis_nominal = np.array(params['grasp']['grasp_nominal_direction'])
        grasp_data.rotate_grasp_frame_to_nominal(grasp_z_axis_nominal)

        self.state.grasp_data = grasp_data
        return_data['grasp_data'] = grasp_data

        if self.debugMode:
            # visualize the grasp frame
            self.visualize_grasp(grasp_data)

        return True, return_data


    def start_bagging(self):
        print "Waiting for 'start_bagging_fusion_data' service..."
        rospy.wait_for_service('start_bagging_fusion_data')
        print "Found it!, starting bagging..."
        try:
            start_bagging_fusion_data = rospy.ServiceProxy('start_bagging_fusion_data', StartBaggingFusionData)
            resp1 = start_bagging_fusion_data()
            # return resp1.data_filepath
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def stop_bagging(self):
        print "Waiting for 'stop_bagging_fusion_data' service..."
        rospy.wait_for_service('stop_bagging_fusion_data')
        print "Found it!, stopping bagging..."
        try:
            stop_bagging_fusion_data = rospy.ServiceProxy('stop_bagging_fusion_data', StopBaggingFusionData)
            resp1 = stop_bagging_fusion_data()
            return resp1.status
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def testInThread(self):
        """
        DEPRECATED
        Runs the grasping pipeline
        1. Move the robot to collect sensor data
        2. Request the grasp (via  a Ros Action)
        3. Move Home
        4. Wait for the response from SpartanGrasp
        5. Process the result
        """

        self.collectSensorData()
        self.moveHome()
        self.requestGrasp()
        result = self.waitForGenerateGraspsResult()
        graspFound = self.processGenerateGraspsResult(result)
        return graspFound




    def testMoveHome(self):
        self.taskRunner.callOnThread(self.moveHome)

    def test(self):
        self.taskRunner.callOnThread(self.testInThread)

    def test_grasp_3D_location(self):
        """
        Calls grasp_3D_location in a thread
        :return:
        """
        self.taskRunner.callOnThread(self.grasp_3D_location)

    def testAttemptGrasp(self):
        self.taskRunner.callOnThread(self.attemptGrasp, self.graspFrame)

    def testPickupObject(self):
        self.taskRunner.callOnThread(self.pickupObject)

    def test_pickup_object(self):
        self.taskRunner.callOnThread(self.pickup_object)

    def testGraspAndStowObject(self):
        self.taskRunner.callOnThread(self.graspAndStowObject)

    def testPipeline(self):
        self.taskRunner.callOnThread(self.planGraspAndPickupObject)

    def testCollectSensorData(self, **kwargs):
        self.taskRunner.callOnThread(self.collectSensorData, **kwargs)

    def testRequestGrasp(self):
        self.taskRunner.callOnThread(self.requestGrasp)

    def testInteractionLoop(self, num_interactions=3):
        self.taskRunner.callOnThread(self.interactAndCollectFusionDataLoop, num_interactions)

    def test_on_clicked_point(self):
        self.taskRunner.callOnThread(self.on_clicked_point)

    def testFindBestMatch(self):
        self.taskRunner.callOnThread(self.findBestBatch)

    def test_grasp_best_match(self):
        self.taskRunner.callOnThread(self.grasp_best_match)

    def test_find_best_match_and_grasp_and_stow(self):
        self.taskRunner.callOnThread(self.find_best_match_and_grasp_and_stow)

    def test_best_match_no_data(self):
        self.taskRunner.callOnThread(self.request_best_match)

    def test_reorient(self):
        self.taskRunner.callOnThread(self.pickup_object_and_reorient_on_table)

    def test_interact_with_object(self):
        self.taskRunner.callOnThread(self.interact_with_object)

    def test_start_bagging(self):
        self.taskRunner.callOnThread(self.start_bagging)

    def test_stop_bagging(self):
        self.taskRunner.callOnThread(self.stop_bagging)

    def test_execute_grasp(self):
        self.taskRunner.callOnThread(self.execute_grasp)

    def test_request_spartan_grasp(self, *args, **kwargs):
        """
        Collect sensor data and send request to spartan_grasp
        Visualize resulting grasp
        :return:
        """
        self.taskRunner.callOnThread(self.request_spartan_grasp, *args, **kwargs)

    def test_run_poser(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_poser, *args, **kwargs)

    def test_run_manipulate_object(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_manipulate_object, *args, **kwargs)

    def test_run_category_manipulation_goal_estimation(self,*args, **kwargs):
        self.taskRunner.callOnThread(self.run_category_manipulation_goal_estimation, *args, **kwargs)

    def test_run_category_manipulation_pipeline(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_category_manipulation_pipeline, *args, **kwargs)

    def test_run_keypoint_detection(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_keypoint_detection, *args, **kwargs)

    def test_run_mug_on_rack_manipulation(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_mug_on_rack_manipulation, *args, **kwargs)

    def test_retract_from_rack(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.retract_from_mug_rack, *args, **kwargs)

    def test_retract_from_mug_shelf(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.retract_from_mug_shelf, *args, **kwargs)

    def test_run_mug_shelf_manipulation(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_mug_shelf_manipulation, *args, **kwargs)

    def test_run_shoe_manipulation(self, *args, **kwargs):
        self.taskRunner.callOnThread(self.run_shoe_rack_manipulation, *args, **kwargs)

    def loadDefaultPointCloud(self):
        self.pointCloudListMsg = GraspSupervisor.getDefaultPointCloudListMsg()

    def test_dev(self):
        def thread_fun():
            self.run_keypoint_detection(wait_for_result=False, move_to_stored_pose=True)
            speed = self.graspingParams['speed']['fast']
            self.moveHome(speed=speed)
            self.run_category_manipulation_goal_estimation()

        self.taskRunner.callOnThread(thread_fun)


    def test_mug_shelf_3D_pipeline(self):
        self.taskRunner.callOnThread(self.run_mug_shelf_3D_pipeline)

    def test_mug_rack_pipeline(self, *args, **kwargs):
        # time.sleep(10.0) # sleep for 10 seconds
        self.taskRunner.callOnThread(self.run_mug_on_rack_pipeline, *args, **kwargs)

    def test_shoe_rack_pipeline(self):
        self.taskRunner.callOnThread(self.run_shoe_on_rack_pipeline)

    def test_category_manip_pipeline(self):
        """
        Runs the appropriate category manip pipeline
        :return:
        :rtype:
        """
        raise NotImplementedError("")

    def test_estimate_mug_rack_pose(self):
        self.taskRunner.callOnThread(self.estimate_mug_rack_pose)

    def r(self):
        self.test_retract_from_rack()

    @staticmethod
    def rectangleMessageFromYamlNode(node):
        msg = spartan_grasp_msgs.msg.Rectangle()
        msg.min_pt = rosUtils.listToPointMsg(node['min_pt'])
        msg.max_pt = rosUtils.listToPointMsg(node['max_pt'])
        msg.pose = rosUtils.ROSPoseMsgFromPose(node)
        return msg

    @staticmethod
    def makeDefault(**kwargs):
        graspingParamsFile = os.path.join(spartanUtils.getSpartanSourceDir(), 'src', 'catkin_projects',
                                          'station_config', 'RLG_iiwa_1', 'manipulation', 'params.yaml')

        return GraspSupervisor(graspingParamsFile=graspingParamsFile, **kwargs)

    @staticmethod
    def getPointCloudListMsg(rosBagFilename):
        bag = rosbag.Bag(rosBagFilename)
        pointCloudListMsg = None
        for topic, msg, t in bag.read_messages(topics=['data']):
            pointCloudListMsg = msg
        bag.close()
        return pointCloudListMsg

    @staticmethod
    def getDefaultPointCloudListMsg():
        spartanSourceDir = spartanUtils.getSpartanSourceDir()

        # filename = "grasp_sensor_data.bag"
        filename = "sr300_box.bag"

        rosBagFilename = os.path.join(spartanSourceDir, 'data', 'rosbag', 'iiwa', filename)

        return GraspSupervisor.getPointCloudListMsg(rosBagFilename)
