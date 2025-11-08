import pygame
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import numpy as np

class MecanumRobotController(Node):
    def __init__(self):
        super().__init__('slam_robot_controller')
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No controller detected!")
            return

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected to {self.joystick.get_name()}")

        self.running = True
        self.controller_thread = threading.Thread(target=self.controller_loop, daemon=True)
        self.controller_thread.start()

    def apply_deadzone(self, value: float, deadzone: float = 0.1) -> float:
        """Filter out small joystick movements to prevent drift."""
        return value if abs(value) > deadzone else 0.0

    def scale_to_range(self, value: float, min_val: float = 0.0, max_val: float = 2.0) -> float:
        """Scale joystick input to be between min_val and max_val while keeping direction."""
        if value == 0.0:
            return 0.0  # Keep zero input at zero
        sign = 1.0 if value > 0.0 else -1.0
        scaled_value = min_val + (abs(value) * (max_val - min_val))  # Scale within range
        return sign * min(scaled_value, max_val)  # Ensure it doesn't exceed max_val

    def controller_loop(self):
        while self.running:
            try:
                pygame.event.pump()

                # Read joystick values and apply deadzone filtering
                right_x = self.apply_deadzone(-self.joystick.get_axis(1))   # Strafe
                # right_y = self.apply_deadzone(-self.joystick.get_axis(0)) 
                # print(right_x)
                # print(right_y) # Forward/Backward
                left_x = self.apply_deadzone(-self.joystick.get_axis(4))    # Rotation
                # left_y = self.apply_deadzone(-self.joystick.get_axis(3)) 
                # print(left_x)
                # print(left_y)  # Extra Y-axis (if needed)

                w1 = right_x
                w2 = right_x
                w3 = left_x
                w4 = left_x

                # print("wheel 1 :" , w1)
                # print("wheel 2 :" , w2)
                # print("Wheel 3 :" , w3) 
                # print("Wheel 4 :" , w4) 
                #  # Example wheel speeds

                # Define wheel speed vector
                w = np.array([[w1], [w2], [w3], [w4]])
                
                print("Wheel veloities :" , w)

                # Robot dimensions
                d = 0.153 / 2
                a = 0.022 / 2  # Ensure a ≠ 0

                # Transformation matrix
                W = np.array([[1/a, 0, -(d)/a],
                            [1/a,  0, -(d)/a],
                            [1/a, 0,  (d)/a],
                            [1/a,  0,  (d)/a]])

                # Use pseudo-inverse if W is not square
                W_inv = np.linalg.pinv(W)

                # Compute motion vector (x, y, rotation)
                x = np.matmul(W_inv, w)
                
                print("uvr =" ,x)


                





                # # Mecanum Wheel Control (same logic as before)
                # # forward = (right_y + left_y)
                # # strafe = -1.0 * (right_x + left_x)
                # # rotation = (left_y - right_y)

                # # # Scale values to be within 4.0 to 10.0 (as floating-point)
                # # forward = self.scale_to_range(forward)
                # # strafe = self.scale_to_range(strafe)
                # # rotation = self.scale_to_range(rotation)

                # # # Compute wheel speeds
                # # fl = forward + strafe + rotation  # Front Left
                # # fr = forward - strafe - rotation  # Front Right
                # # bl = forward - strafe + rotation  # Back Left
                # # br = forward + strafe - rotation  # Back Right

                # # # Normalize values to keep them within -1.0 to 1.0
                # # max_val = max(abs(fl), abs(fr), abs(bl), abs(br))
                # # if max_val > 1.0:
                # #     fl /= max_val
                # #     fr /= max_val
                # #     bl /= max_val
                # #     br /= max_val
                
                u = self.scale_to_range(x[0][0])
                v = self.scale_to_range(x[1][0])
                r = self.scale_to_range(x[2][0])

                # Publish Twist message
                twist_msg = Twist()
                twist_msg.linear.x = float(u) * 20
                # twist_msg.linear.y = float(v) * 2
                twist_msg.angular.z = float(r) * 20

                self.vel_publisher.publish(twist_msg)
                # self.get_logger().info(f"Published Twist: {twist_msg}")

                time.sleep(0.5)

            except Exception as e:
                self.get_logger().error(f"Error in controller loop: {e}")
                self.running = False

    def destroy_node(self):
        """Ensure thread stops when shutting down"""
        self.running = False
        if self.controller_thread.is_alive():
            self.controller_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MecanumRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
