#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import argparse
from moveit_msgs.msg import CollisionObject, OrientationConstraint, Constraints
from shape_msgs.msg import SolidPrimitive

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Move Sawyer to a specific pose using MoveIt')
    parser.add_argument('-p', '--position', nargs=3, type=float, required=True,
                        help='Position (x y z) in meters')
    parser.add_argument('-o', '--orientation', nargs=4, type=float, required=True,
                        help='Orientation quaternion (x y z w)')
    parser.add_argument('--planning-time', type=float, default=5.0,
                        help='Planning time in seconds (default: 5.0)')
    parser.add_argument('--max-velocity', type=float, default=0.2,
                        help='Maximum velocity scaling factor (default: 0.2)')
    
    args = parser.parse_args()

    # Initialize moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_sawyer_pose_goal', anonymous=True)

    # Instantiate objects for robot, scene, and group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "right_arm"  # Check this matches your Sawyer MoveIt setup
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Allow more orientation flexibility

    move_group.set_goal_orientation_tolerance(0.5)  # yaw

    # Create a DisplayTrajectory publisher for RViz visualization
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    # Print useful info
    planning_frame = move_group.get_planning_frame()
    print("Planning frame:", planning_frame)

    eef_link = move_group.get_end_effector_link()
    print("End effector link:", eef_link)
    # Set pose reference frame to the robot's wrist (end effector link)
    move_group.set_pose_reference_frame(eef_link)

    group_names = robot.get_group_names()
    print("Available Planning Groups:", group_names)

    print("Robot state:")
    print(robot.get_current_state())

    ## --------------------
    ## Add Floor Obstacle (z = -0.12)
    ## --------------------
    print("Adding floor obstacle at z = -0.12...")
    
    # Create a collision object for the floor
    floor = CollisionObject()
    floor.header.frame_id = "base"
    floor.id = "floor_obstacle"
    
    # Create a box primitive for the floor (large enough to cover workspace)
    floor_primitive = SolidPrimitive()
    floor_primitive.type = SolidPrimitive.BOX
    floor_primitive.dimensions = [2.0, 2.0, 0.1]  # length, width, height
    
    # Set the floor pose (positioned so top surface is at z = -0.12)
    floor_pose = geometry_msgs.msg.Pose()
    floor_pose.position.x = 0.0
    floor_pose.position.y = 0.0
    floor_pose.position.z = -0.27  # Half height so top surface is at -0.22
    floor_pose.orientation.w = 1.0  # No rotation
    
    # Add the primitive and pose to the collision object
    floor.primitives.append(floor_primitive)
    floor.primitive_poses.append(floor_pose)
    
    # Add the floor to the scene
    scene.add_object(floor)
    
    # Wait a moment for the scene to update
    rospy.sleep(1.0)
    print("Floor obstacle added successfully!")

    ## --------------------
    ## Add Ceiling Obstacle (z = 1.0)
    ## --------------------
    print("Adding ceiling obstacle at z = 1.0...")
    
    # Create a collision object for the ceiling
    ceiling = CollisionObject()
    ceiling.header.frame_id = "base"
    ceiling.id = "ceiling_obstacle"
    
    # Create a box primitive for the ceiling (large enough to cover workspace)
    ceiling_primitive = SolidPrimitive()
    ceiling_primitive.type = SolidPrimitive.BOX
    ceiling_primitive.dimensions = [2.0, 2.0, 0.1]  # length, width, height
    
    # Set the ceiling pose (positioned so bottom surface is at z = 1.0)
    ceiling_pose = geometry_msgs.msg.Pose()
    ceiling_pose.position.x = 0.0
    ceiling_pose.position.y = 0.0
    ceiling_pose.position.z = 1.05  # Half height so bottom surface is at 1.0
    ceiling_pose.orientation.w = 1.0  # No rotation
    
    # Add the primitive and pose to the collision object
    ceiling.primitives.append(ceiling_primitive)
    ceiling.primitive_poses.append(ceiling_pose)
    
    # Add the ceiling to the scene
    scene.add_object(ceiling)
    
    # Wait a moment for the scene to update
    rospy.sleep(1.0)
    print("Ceiling obstacle added successfully!")

    ## --------------------
    ## Set End Effector Pose Goal
    ## --------------------

    pose_goal = geometry_msgs.msg.Pose()

    # Set position from command line arguments
    x, y, z = args.position
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    # Set orientation from command line arguments
    pose_goal.orientation.x = args.orientation[0]
    pose_goal.orientation.y = args.orientation[1]
    pose_goal.orientation.z = args.orientation[2]
    pose_goal.orientation.w = args.orientation[3]

    print(f"Moving to position: ({args.position[0]:.3f}, {args.position[1]:.3f}, {args.position[2]:.3f})")
    print(f"With orientation: ({args.orientation[0]:.3f}, {args.orientation[1]:.3f}, {args.orientation[2]:.3f}, {args.orientation[3]:.3f})")

    move_group.set_pose_target(pose_goal)

    # --- Orientation constraint for yaw flexibility ---
    oc = OrientationConstraint()
    oc.link_name = move_group.get_end_effector_link()
    oc.header.frame_id = move_group.get_planning_frame()
    oc.orientation = pose_goal.orientation  # Use the pose goal's orientation as reference
    oc.absolute_x_axis_tolerance = 0.05   # Tight roll
    oc.absolute_y_axis_tolerance = 0.05   # Tight pitch
    oc.absolute_z_axis_tolerance = 0.5    # Flexible yaw (about Z, ±28.6°)
    oc.weight = 1.0
    constraints = Constraints()
    constraints.orientation_constraints.append(oc)
    move_group.set_path_constraints(constraints)
    move_group.set_planning_time(20)

    ## --------------------
    ## Plan to the Goal
    ## --------------------
    print("Planning to pose goal...")

    plan_result = move_group.plan()
    if isinstance(plan_result, tuple):
        success, plan, planning_time, error_code = plan_result
    else:
        plan = plan_result
        success = True  # Assume success for older API

    if not success or plan is None:
        print("Planning failed!")
        return

    ## --------------------
    ## Visualize the Plan in RViz
    ## --------------------
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    if plan and hasattr(plan, 'joint_trajectory'):
        display_trajectory.trajectory.append(plan)
        print("Publishing trajectory to RViz...")
        display_trajectory_publisher.publish(display_trajectory)
        print("Trajectory published. Waiting 5 seconds for visualization...")
        rospy.sleep(5)  # Longer pause to visualize
    else:
        print("Plan does not have a joint_trajectory. Skipping visualization.")

    ## --------------------
    ## Execute the Plan
    ## --------------------
    print("Executing the plan...")
    move_group.go(wait=True)

    # Clear path constraints after execution
    move_group.clear_path_constraints()

    # Ensure no residual movement
    move_group.stop()
    move_group.clear_pose_targets()

    ## --------------------
    ## Done
    ## --------------------
    print("Task complete.")

    rospy.sleep(1)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main() 