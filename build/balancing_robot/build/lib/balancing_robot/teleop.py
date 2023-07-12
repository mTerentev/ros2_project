import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from pynput import keyboard

class Teleop(Node):

    lin = 0.0
    ang = 0.0

    def __init__(self):
        super().__init__("teleop_node")

        self.pub = self.create_publisher(Twist, "balancing_robot/teleop_signal", 10)

        self.create_timer(0.1, self.callback)

        # ...or, in a non-blocking fashion:
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release = self.on_release)
        listener.start()

    def callback(self):

        msg = Twist()

        msg.linear.x = self.lin
        msg.angular.z = self.ang

        self.pub.publish(msg)

    def on_press(self, key):
        cmd = None
        try:
            print('alphanumeric key {0} pressed'.format(
                key.char))
            cmd = key.char
            
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            
        match cmd:
            case 'w' : self.lin =  4.0
            case 's' : self.lin = -4.0
            case 'a' : self.ang =  4.0
            case 'd' : self.ang = -4.0
        
    def on_release(self, key):
        cmd = None
        try:
            print('alphanumeric key {0} pressed'.format(
                key.char))
            cmd = key.char
            
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            
        match cmd:
            case 'w' : self.lin = 0.0
            case 's' : self.lin = 0.0
            case 'a' : self.ang = 0.0
            case 'd' : self.ang = 0.0

def main():

    rclpy.init()

    teleop = Teleop()

    rclpy.spin(teleop)

    rclpy.shutdown()

if __name__ == "__main__":
    main()