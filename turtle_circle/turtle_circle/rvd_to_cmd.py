import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32
from msg_srv_action_interface.msg import RVD
from geometry_msgs.msg import Twist, Vector3

class RvdToCmd(Node):

    def __init__(self):
        self.radius = 1.0
        self. velocity = 0.0
        self. direction = True
        self.linear = None
        self.angular = None
        self.twist = None
        self.linear_x = 0.0
        self.angular_z = 0.0

        super().__init__('rvd_to_cmd')
        qos_profile = QoSProfile(depth=10)

        self.rvd_sub = self.create_subscription(
            RVD,
            'rad_vel_dir',
            self.subscribe_rvd_message,
            qos_profile)
        
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            qos_profile)
        self.timer = self.create_timer(1, self.publish_cmd_msg)
        self.count = 0
    
    def subscribe_rvd_message(self, msg):
        self.radius = msg.radius
        self.velocity = msg.velocity
        self.direction = msg.direction

    def publish_cmd_msg(self):
        self.twist = Twist()
        self.linear = Vector3()
        self.angular = Vector3()

        self.linear_x = self.velocity
        if self.direction == True:
            self.angular_z = self.velocity / (self.radius + 0.000001)
        elif self.direction != True:
            self.angular_z = - (self.velocity / (self.radius + 0.000001))
        
        self.linear.x = self.linear_x
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = self.angular_z
        self.twist.linear = self.linear
        self.twist.angular = self.angular

        self.cmd_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    node = RvdToCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrypt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()