import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import JointState

class GraspingNode(Node):
    def __init__(self):
        super().__init__('grasping_node')

        # Declare parameters
        self.declare_parameter('qf', [math.pi/4, math.pi/4, math.pi/4, 0.0, 0.0, 0.0])
        self.declare_parameter('finger_qf', [0.0, -1.0, -0.5, 0.0, -1.0, -0.5, 0.0, -1.0, -0.5])
        self.declare_parameter('n_T', 20)

        # Retrieve parameters
        self.qf = self.get_parameter('qf').get_parameter_value().double_array_value
        self.finger_qf = self.get_parameter('finger_qf').get_parameter_value().double_array_value
        self.n_T = self.get_parameter('n_T').get_parameter_value().integer_value

        # RViz animation variables and publisher for JointState messages
        self.animation_step = 0
        self.joint_names = [
            "finger1_joint0", "finger1_joint1", "finger1_joint2",
            "finger2_joint0", "finger2_joint1", "finger2_joint2",
            "finger3_joint0", "finger3_joint1", "finger3_joint2",
                        "grasp_box_trans_x", "grasp_box_trans_y", "grasp_box_trans_z",
            "grasp_box_rot_x",   "grasp_box_rot_y",   "grasp_box_rot_z",
        ]
        self.finger_joint_defaults = {
            1: [0.0, -1.1781, -0.4908],
            2: [0.0, -1.1781, -0.4908],
            3: [0.0, -1.1781, -0.4908]
        }
        self.current_finger_joints = []
        for finger in [1, 2, 3]:
            self.current_finger_joints.extend(self.finger_joint_defaults[finger])

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for algorithm implementation
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.animate_system()
    
    def animate_system(self):
        if self.animation_step > self.n_T:
            self.get_logger().info("All transformations animated.")
            rclpy.shutdown()
            return
        
        interpolated_finger = []
        for i in range(len(self.current_finger_joints)):
            default_value = self.finger_joint_defaults[(i // 3) + 1][i % 3]
            target_value  = self.finger_qf[i]
            interpolation_value = default_value + (target_value - default_value) * (self.animation_step / self.n_T)
            interpolated_finger.append(interpolation_value)
        
        box_rotations = [(self.qf[i] * self.animation_step) / self.n_T for i in range(0, 3)]
        box_translations = [(self.qf[i+3] * self.animation_step) / self.n_T for i in range(0, 3)]
        box_transformations =  box_translations + box_rotations

        full_joint_values = interpolated_finger + box_transformations 

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = full_joint_values
        self.joint_state_pub.publish(js)
        self.get_logger().info(f"Animation step {self.animation_step}: Published JointState: {full_joint_values}")
        
        self.animation_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = GraspingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()