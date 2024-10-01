import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading

class Move(Node):
    def __init__(self):
        super().__init__('xyz_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.get_logger().info("Move node initialized and ready to publish target positions.")

    def move_xyz(self, xyz):
        try:
            if len(xyz) != 3:
                self.get_logger().error("3 input values required for Move (x y z).")
                return
            msg = Float32MultiArray()
            msg.data = xyz
            self.publisher.publish(msg)
            self.get_logger().info(f"Published Target Position: {msg.data}")
        except ValueError as e:
            self.get_logger().error(f"Invalid input for Move: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in Move: {e}")

class Gripper(Node):
    def __init__(self):
        super().__init__('grip_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, '/gripper_angle', 10)
        self.get_logger().info("Gripper node initialized and ready to publish gripper angles.")

    def set_gripper(self, angle):
        try:
            if not isinstance(angle, (int, float)):
                self.get_logger().error("Gripper angle must be a number.")
                return
            msg = Float32MultiArray()
            msg.data = [float(angle)]
            self.publisher.publish(msg)
            self.get_logger().info(f"Published Gripper Angle: {msg.data[0]}")
        except ValueError as e:
            self.get_logger().error(f"Invalid input for Gripper: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in Gripper: {e}")

class Interface(Node):
    def __init__(self):
        super().__init__('interface_node')
        self.move_node = Move()
        self.gripper_node = Gripper()
        self.get_logger().info("Interface node started. Awaiting commands...")

        # Start a separate thread to handle user input
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True  # Allows thread to be killed when main program exits
        self.input_thread.start()

    def get_user_input(self):
        while rclpy.ok():
            try:
                user_input = input("Enter command (Move x y z | Gripper angle): ")
                if user_input.strip() == "":
                    continue  # Ignore empty input
                parts = user_input.strip().split()

                if parts[0].lower() == "move":
                    if len(parts) != 4:
                        self.get_logger().error("Invalid Move command. Usage: Move x y z")
                        continue
                    try:
                        x, y, z = map(float, parts[1:4])
                        self.move_node.move_xyz([x, y, z])
                    except ValueError:
                        self.get_logger().error("Move command requires three numerical values.")
                
                elif parts[0].lower() == "grip":
                    if len(parts) != 2:
                        self.get_logger().error("Invalid Gripper command. Usage: Gripper angle")
                        continue
                    try:
                        angle = float(parts[1])
                        self.gripper_node.set_gripper(angle)
                    except ValueError:
                        self.get_logger().error("Gripper command requires a numerical value for angle.")
                
                else:
                    self.get_logger().error("Unknown command. Use 'Move x y z' or 'Gripper angle'.")
            
            except EOFError:
                self.get_logger().info("EOF detected. Shutting down Interface node.")
                rclpy.shutdown()
            except Exception as e:
                self.get_logger().error(f"Error processing input: {e}")

def main(args=None):
    rclpy.init(args=args)
    interface = Interface()
    
    # Create a MultiThreadedExecutor to handle multiple nodes concurrently
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(interface)
    executor.add_node(interface.move_node)
    executor.add_node(interface.gripper_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        interface.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        # Cleanly shut down all nodes
        executor.shutdown()
        interface.move_node.destroy_node()
        interface.gripper_node.destroy_node()
        interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
