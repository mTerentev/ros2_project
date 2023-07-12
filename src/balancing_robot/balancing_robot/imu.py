import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class IMU_Decoder(Node):

    def __init__(self):
        super().__init__("imu_decoder")

        self.create_subscription(Imu, "balancing_robot/imu/out", self.callback, 10)

        self.pub = self.create_publisher(Float64, "balancing_robot/tilt", 10)

    def callback(self, msg : Imu):

        # data : float = msg.orientation.y

        q = msg.orientation

        roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)

        print(pitch)

        message = Float64()

        message.data = pitch

        self.pub.publish(message)
 
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def main():

    rclpy.init()

    node = IMU_Decoder()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()