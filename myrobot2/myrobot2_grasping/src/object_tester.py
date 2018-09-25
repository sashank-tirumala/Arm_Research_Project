#!/usr/bin/env python
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown
from actionlib import SimpleActionClient, GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions
from tf.transformations import quaternion_from_euler
import sys
import copy
import numpy
# Create dict with human readable MoveIt! error codes:
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class Pick_Place:
    def __init__(self):
        # Retrieve params:
        self._table_object_name = rospy.get_param('~table_object_name', 'Grasp_Table')
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'Grasp_Object')

        self._grasp_object_width = rospy.get_param('~grasp_object_width', 0.01)

        self._arm_group     = rospy.get_param('~manipulator', 'manipulator')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')

        self._approach_retreat_desired_dist = rospy.get_param('~approach_retreat_desired_dist', 0.2)
        self._approach_retreat_min_dist = rospy.get_param('~approach_retreat_min_dist', 0.1)

        # Create (debugging) publishers:
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        self._places_pub = rospy.Publisher('places', PoseArray, queue_size=1, latch=True)

        # Create planning scene and robot commander:
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        # Retrieve groups (arm and gripper):
        self._arm     = self._robot.get_group(self._arm_group)
        self._gripper = self._robot.get_group(self._gripper_group)


        rospy.sleep(1.0)

        # self.setup()
        # Clean the scene:
        self._scene.remove_world_object(self._table_object_name)
        self._scene.remove_world_object(self._grasp_object_name)

        # Add table and Coke can objects to the planning scene:
        self._pose_table    = self._add_table(self._table_object_name)
        rospy.loginfo("added table")
        rospy.sleep(2.0)
        self._pose_coke_can = self._add_grasp_block_(self._grasp_object_name)
        rospy.loginfo("added grasping box")
        rospy.sleep(1.0)

        # Define target place pose:
        self._pose_place = Pose()

        self._pose_place.position.x = self._pose_coke_can.position.x
        self._pose_place.position.y = self._pose_coke_can.position.y - 0.06
        self._pose_place.position.z = self._pose_coke_can.position.z

        self._pose_place.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))

    def _add_table(self, name):
        rospy.loginfo("entered table")
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -0.2

        q = quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
        p.pose.orientation = Quaternion(*q)

        # Table size from ~/.gazebo/models/table/model.sdf, using the values
        # for the surface link.
        self._scene.add_box(name, p, (9 , 9, 0.02))

        return p.pose

    def _add_grasp_block_(self, name):
        rospy.loginfo("entered box grabber")
        p = PoseStamped()
        p.header.frame_id = self._robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = 0.5
        p.pose.position.y = 0.05
        p.pose.position.z = 0.32

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/coke_can.dae,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the coke cylinder.
        # The values are taken from the cylinder base diameter and height.
        self._scene.add_box(name, p, (0.1, 0.1, 0.1))

        return p.pose

def main():
    p = Pick_Place()

    rospy.spin()

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place')
    main()

roscpp_shutdown()
