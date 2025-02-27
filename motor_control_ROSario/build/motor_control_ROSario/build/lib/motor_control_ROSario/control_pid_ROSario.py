import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class PIDController(Node):
    def __init__(self):
        super().__init__('control_pid_ROSario')

        # Parámetros del PID
        self.declare_parameter('kp', 0.784448352257978)
        self.declare_parameter('ki', 18.9480680587262)
        self.declare_parameter('kd', 0.00259088726253201)

        # Variables del PID
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.01  # Tiempo de muestreo

        # Suscriptores
        self.subscription_setpoint = self.create_subscription(Float32, '/set_point_ROSario', self.setpoint_callback, 10)
        self.subscription_output = self.create_subscription(Float32, '/motor_speed_y_ROSario', self.output_callback, 10)

        # Publicador
        self.publisher = self.create_publisher(Float32, '/motor_input_u_ROSario', 10)

        # Variables de control
        self.set_point = 0.0
        self.motor_output = 0.0
        self.u = 0.0

        self.received_setpoint = False
        self.received_output = False

        # Timer para ejecutar el control a intervalos regulares
        self.timer = self.create_timer(self.dt, self.control_loop)

    def setpoint_callback(self, msg):
        self.set_point = msg.data
        self.received_setpoint = True

    def output_callback(self, msg):
        self.motor_output = msg.data
        self.received_output = True

    def control_loop(self):    
        # Cálculo del error
        error = self.set_point - self.motor_output

        # Términos del PID
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        # Señal de control
        u = self.kp * error + self.ki * self.integral + self.kd * derivative
                
        # Publicar señal de control
        msg = Float32()
        msg.data = u
        self.publisher.publish(msg)

        # Almacenar error anterior
        self.prev_error = error

        # Actualizar parámetros en caso de que cambien
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

# Main
def main(args=None):
    rclpy.init(args=args)

    control_pid = PIDController()

    try:
        rclpy.spin(control_pid)
    except KeyboardInterrupt:
        pass
    finally:
        control_pid.destroy_node()
        rclpy.try_shutdown()

#Execute Node
if __name__ == '__main__':
    main()
