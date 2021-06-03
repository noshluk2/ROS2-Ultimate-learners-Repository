import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def timed_callback():
    global node ,pub
    msg = Twist()
    msg.linear.x =2.0
    msg.angular.z=1.0
    pub.publish(msg)
    
    

def main(args=None):
    rclpy.init(args=args)

    global node, pub
    node = Node('custom_cmd_vel_node')
    pub=node.create_publisher(Twist,'/turtle1/cmd_vel',10)
    node.create_timer(2, timed_callback)

    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()