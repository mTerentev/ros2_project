import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

def main():
    rclpy.init(args=None)

    model=open("install/balancing_robot/share/balancing_robot/sdf/balancing_robot.sdf", "r")

    node = Node("spawner")
    client = node.create_client(SpawnEntity, "/spawn_entity")

    while(not client.service_is_ready()):
        pass

    request = SpawnEntity.Request()
    request.name = "balancing_robot"
    request.xml = model.read()

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()