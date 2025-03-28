import rclpy
from rclpy.node import Node
import math
import numpy as np

from sensor_msgs.msg import JointState

class GraspingNode(Node):
    def __init__(self):
        super().__init__('grasping_node')

        # Declare parameters
        self.declare_parameter('box_qi', [math.pi/4, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('box_qf', [math.pi/4, math.pi/4, math.pi/4, 0.0, 0.0, 0.0])
        self.declare_parameter('n_T', 20)

        # Retrieve parameters and convert to numpy arrays
        self.box_qi = np.array(list(self.get_parameter('box_qi').get_parameter_value().double_array_value))
        self.box_qf = np.array(list(self.get_parameter('box_qf').get_parameter_value().double_array_value))
        self.n_T = self.get_parameter('n_T').get_parameter_value().integer_value

        # Full system joint names (9 finger joints + 6 for the grasp box)
        self.system_joint_names = [
            "finger1_joint0", "finger1_joint1", "finger1_joint2",
            "finger2_joint0", "finger2_joint1", "finger2_joint2",
            "finger3_joint0", "finger3_joint1", "finger3_joint2",
            "grasp_box_trans_x", "grasp_box_trans_y", "grasp_box_trans_z",
            "grasp_box_rot_x",   "grasp_box_rot_y",   "grasp_box_rot_z",
        ]

        # Create publisher for joint_states
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Trajectory decomposition initialization
        self.current_T = 0
        self.current_box_q = np.copy(self.box_qi)

        # Timer for algorithm implementation (0.1 sec per iteration)
        timer_period = 0.1  
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        # Check if box trajectory is complete
        if self.current_T > self.n_T:
            self.get_logger().info("Box trajectory completed.")
            self.timer.cancel()  # Stop the timer
            return
        
        # Compute current_box_q as linear interpolation from box_qi to box_qf
        fraction = self.current_T / self.n_T
        self.current_box_q = self.box_qi + (self.box_qf - self.box_qi) * fraction

        self.current_T += 1
        
def main(args=None):
    rclpy.init(args=args)
    node = GraspingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
