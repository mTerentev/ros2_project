import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class Regulator(Node):

    dt = 0.01
    target = 0
    value = 0
    previous_err = 0
    int = 0
    control = 0

    def __init__(self):
        super().__init__("pid_regulator_node")

        self.declare_parameter("P", 0.0)
        self.declare_parameter("I", 0.0)
        self.declare_parameter("D", 0.0)
        self.declare_parameter("K", 0.0)
        self.declare_parameter("value_topic", "")
        self.declare_parameter("control_topic", "")
        self.declare_parameter("target_topic", "")

        self.P = self.get_parameter("P").value
        self.I = self.get_parameter("I").value
        self.D = self.get_parameter("D").value
        self.K = self.get_parameter("K").value

        self.create_subscription(Float64, self.get_parameter("value_topic").value, self.storeValue, 10)

        self.create_subscription(Float64, self.get_parameter("target_topic").value, self.storeTarget, 10)

        self.pub = self.create_publisher(Float64, self.get_parameter("control_topic").value, 10)

        self.create_timer(self.dt, self.callback)

    def storeValue(self, msg : Float64):
        self.value = msg.data

    def storeTarget(self, msg : Float64):
        self.target = msg.data

    def odometryProcess(self, msg : Odometry):
        self.vel = msg.twist.twist.linear.x

    def callback(self):

        err = self.value - self.target

        self.dif = (err - self.previous_err) / self.dt
        self.int += err * self.dt

        self.control = (self.P * err + self.I * self.int + self.D * self.dif) * self.K

        msg = Float64()
        msg.data = self.control

        self.pub.publish(msg)

        self.previous_err = err

        self.P = self.get_parameter("P").value
        self.I = self.get_parameter("I").value
        self.D = self.get_parameter("D").value
        self.K = self.get_parameter("K").value

def main():
    rclpy.init()

    node = Regulator()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__name__":
    main()