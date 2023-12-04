import rclpy
from rclpy.node import Node
import moveit
from moveit import MoveGroupInterface
from geometry_msgs.msg import Pose, Quaternion, Point

def main(args=None):
    rclpy.init(args=args)
    node = Node("hello_moveit")
    node.get_logger().info("Hi from hello_moveit.")
    # Next step goes here

    # A MoveGroupInterface wraps the C++ MoveGroupInterface.
    # It creates a node called "hello_moveit_interface" that runs in the background and
    # communicates (via publishers/subscribers) with an already running move_group node
    move_group_interface=MoveGroupInterface("hello_moveit_interface", "manipulator")

    target_pose = Pose(quaternion=Quaternion(w=1),
                    position=Point(x=0.28, y=-0.2, z=0.5))

    # Set the target pose
    move_group_interface.setPoseTarget(target_pose)

    # Plan the motion
    (result, plan) = move_group_interface.plan()

    # Execute the motion if planning was successful
    if result.val == result.SUCCESS:
        move_group_interface.execute(plan)
    else:
        node.get_logger().error("Planning Failed")


if __name__ == '__main__':
    main()

