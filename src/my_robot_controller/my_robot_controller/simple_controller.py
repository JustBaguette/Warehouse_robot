import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import numpy as np
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius",0.1)
        self.declare_parameter("wheel_separation",0.45)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("using wheel_radius " + str(self.wheel_radius))
        self.get_logger().info("using wheel_separation " + str(self.wheel_separation))

        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10 )
        self.pubber = self.create_publisher(TwistStamped, "mobile_robot_controller/cmd_vel",10)
        self.speed_conversion = np.array(([self.wheel_radius/2,self.wheel_radius/2],[self.wheel_radius/self.wheel_separation,-self.wheel_radius/self.wheel_separation]))
        self.get_logger().info("The conversion matrix is " + str(self.speed_conversion))
        

    def velCallBack(self,msg):
        robot_speed = np.array(([msg.twist.linear.x],[msg.twist.angular.z]))

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion), robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[0,0],wheel_speed[0,1]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()