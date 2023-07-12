import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Controller(Node):

    target : float = 0.0
    velocity : float = 0.0
    steering : float = 0.0

    vel_arr = []

    def __init__(self):
        super().__init__("controller")

        self.create_subscription(Float64, "balancing_robot/torque", self.callback, 10)

        self.create_subscription(Twist, "balancing_robot/teleop_signal", self.getControl, 10)

        self.create_subscription(Odometry, "balancing_robot/odom", self.getVelocity, 10)


        self.control_pub = self.create_publisher(Float64, "balancing_robot/velocity_control", 10)

        self.pub = self.create_publisher(Twist, "balancing_robot/cmd_vel", 10)

        self.vel_pub = self.create_publisher(Float64, "balancing_robot/velocity", 10)

        self.create_timer(0.1, self.control)

    def callback(self, msg : Float64):

        data : float = msg.data
        message = Twist()
        message.linear.x = data
        message.angular.z = self.steering
        self.pub.publish(message)

    def control(self):

        msg = Float64()
        msg.data = self.target
        self.control_pub.publish(msg)

        msg1 = Float64()
        msg1.data = -self.velocity
        self.vel_pub.publish(msg1)

    def getControl(self, msg : Twist):

        self.target = msg.linear.x
        self.steering = msg.angular.z

    def getVelocity(self, msg : Odometry):

        v = msg.twist.twist.linear.x

        self.velocity += v / 100

        self.vel_arr.append(v)

        if len(self.vel_arr) > 100:
            self.velocity -= self.vel_arr.pop(0)/100


def main():

    rclpy.init()

    node = Controller()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()