#custom_node/sdf_spawner.py
## This node is going to spawn sdf file 
## argments should be provided to it 
## 1st argument -> sdf file path 
## 2nd argument -> name of the model after spawning in gazebo
## 3,4,5th argument -> position of the model in gazebo

import sys
from gazebo_msgs.srv import SpawnEntity
import rclpy


def main():
    # take argument from user while to run the spawning_node
    argv = sys.argv[1:]
    sdf_path = argv[0] # user first input is sdf file path
    rclpy.init()
    # Create a spawning_node
    spawning_node = rclpy.create_node("Sdf_spawning_node")
    # Creating a spawning_node for gazebo based spawn_entity service
    client = spawning_node.create_client(SpawnEntity, "/spawn_entity")
    # waiting for service to be connected
    if not client.service_is_ready():
        client.wait_for_service()
        spawning_node.get_logger().info("Conencted to spawn_entity")
    
    # Sending the request while filling in the user ordered argv
    request = SpawnEntity.Request()
    request.name = argv[1] # user second argument
    request.xml = open(sdf_path, 'r').read() # reading the sdf file
    # Below is the location in x y z co ordinates for the model to be spawn
    request.initial_pose.position.x = float(argv[2])
    request.initial_pose.position.y = float(argv[3])
    request.initial_pose.position.z = float(argv[4])
    
    spawning_node.get_logger().info("Sending service request to `/spawn_entity`")
    # Spinning until model is spawned
    future = client.call_async(request)
    rclpy.spin_until_future_complete(spawning_node, future)
    #  if not spawn then these error and response is generated
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

   
    spawning_node.get_logger().info("SDF file has been spawned in Gazebo")
    spawning_node.destroy_node()
    # node connection shuting down
    rclpy.shutdown()


if __name__ == "__main__":
    main()