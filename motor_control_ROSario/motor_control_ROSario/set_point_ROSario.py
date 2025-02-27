# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

#Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node_ROSario')

        # Retrieve sine wave parameters
        self.declare_parameter('type_flag', 0.0)
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)

        self.type_flag = self.get_parameter('type_flag').value
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
                
        self.timer_period = 0.02 #seconds = 50 Hz

        # Create a publisher and timer for the signal
        self.signal_publisher = self.create_publisher(Float32, 'set_point_ROSario', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        # Create a messages and variables to be used
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("SetPoint Node Started \U0001F680")

    # Timer Callback: Generate and Publish Sine Wave Signal
    def timer_cb(self):
        # Calculate elapsed time
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        # Generate sine wave signal
        if self.type_flag == 0:
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        else:
            self.signal_msg.data = self.amplitude * np.sign(np.sin(self.omega * elapsed_time))
        # Publish the signal
        self.signal_publisher.publish(self.signal_msg)

    def parameters_callback(self, params):
        for param in params:
            # System gain parameter check
            if param.name == "type_flag":
                # Check if it is negative
                if (param.value < 0.0 or param.value > 1.0):
                    self.get_logger().warn("Invalid type_flag! It just can be 0 or 1.")
                    return SetParametersResult(successful=False, reason="type_flag cannot be different from 0 and 1")
                else:
                    self.type_flag = param.value  # Update internal variable
                    self.get_logger().info(f"type_flag updated to {self.type_flag}")
            elif param.name == "amplitude":
                # Check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid amplitude! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="amplitude cannot be negative")
                else:
                    self.amplitude = param.value  # Update internal variable
                    self.get_logger().info(f"amplitude updated to {self.amplitude}")
            elif param.name == "omega":
                # Check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid omega! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="omega cannot be negative")
                else:
                    self.omega = param.value  # Update internal variable
                    self.get_logger().info(f"omega updated to {self.omega}")
        return SetParametersResult(successful=True)
    
def main(args=None):
    rclpy.init(args=args)

    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
