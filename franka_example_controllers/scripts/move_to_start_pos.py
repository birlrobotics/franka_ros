#!/usr/bin/env python3

import sys
import rospy as ros
import ipdb

from actionlib import SimpleActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, \
                             FollowJointTrajectoryGoal, FollowJointTrajectoryResult

# Init node
ipdb.set_trace()
ros.init_node('move_to_start')

# Create an action client that will connect to action topic (i.e. effort_joint_trajectory_controller/follow_joint_trajectory).
action = ros.resolve_name('~follow_joint_trajectory')
client = SimpleActionClient(action, FollowJointTrajectoryAction)

# Verify connection to action server. 
ros.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
client.wait_for_server()

# Launch file loaded goal position via rosparam in franka_control/config/start_pose.yaml
# Access via ros.get_param()
param = ros.resolve_name('~joint_pose')
pose = ros.get_param(param, None)

if pose is None:
    ros.logerr('move_to_start: Could not find required parameter "' + param + '"')
    sys.exit(1)

# Get current robot joint_states...
topic = ros.resolve_name('~joint_states')
ros.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")

# Wait for and pick up exactly one message from joint_states.
joint_state = ros.wait_for_message(topic, JointState)

# Create a dictionary of joint state names and positions 
initial_pose = dict(zip(joint_state.name, joint_state.position))

max_movement = max(abs(pose[joint] - initial_pose[joint]) for joint in pose)

# Create a single point in a trajectory
point = JointTrajectoryPoint()

# Set the time stamp for this point (0.5s)
point.time_from_start = ros.Duration.from_sec(
    # Use either the time to move the furthest joint with 'max_dq' or 500ms,
    # whatever is greater
    max(max_movement / ros.get_param('~max_dq', 0.5), 0.5)
)

# Prepare to add joint names and positions to your goal.
goal = FollowJointTrajectoryGoal()

# Set joint names for goal structure and also point positions.
goal.trajectory.joint_names, point.positions = [list(x) for x in zip(*pose.items())]
point.velocities = [0] * len(pose)

# Add point to goal. 
goal.trajectory.points.append(point)

# Set time error tolerance to goal
goal.goal_time_tolerance = ros.Duration.from_sec(0.5)

# Send the goal
ros.loginfo('Sending trajectory Goal to move into initial config')
client.send_goal_and_wait(goal)

# Get the result
result = client.get_result()

# If error, print the correct error code
if result.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
    
    ros.logerr('move_to_start: Movement was not successful: ' + {
        FollowJointTrajectoryResult.INVALID_GOAL:
        """
        The joint pose you want to move to is invalid (e.g. unreachable, singularity...).
        Is the 'joint_pose' reachable?
        """,

        FollowJointTrajectoryResult.INVALID_JOINTS:
        """
        The joint pose you specified is for different joints than the joint trajectory controller
        is claiming. Does you 'joint_pose' include all 7 joints of the robot?
        """,

        FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED:
        """
        During the motion the robot deviated from the planned path too much. Is something blocking
        the robot?
        """,

        FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED:
        """
        After the motion the robot deviated from the desired goal pose too much. Probably the robot
        didn't reach the joint_pose properly
        """,
    }[result.error_code])

else:
    ros.loginfo('move_to_start: Successfully moved into start pose')
