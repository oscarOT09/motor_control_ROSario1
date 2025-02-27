import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from servicio_ctrl.srv import SwitchCtrl

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
        self.dt = 0.01  # 100 Hz

        # Suscriptores
        self.subscription_setpoint = self.create_subscription(Float32, 'set_point_ROSario', self.setpoint_callback, 10)
        self.subscription_output = self.create_subscription(Float32, 'motor_speed_y_ROSario', self.output_callback, 10)

        # Publicador
        self.publisher = self.create_publisher(Float32, 'motor_input_u_ROSario', 10)

        # Variables de control
        self.set_point = 0.0
        self.motor_output = 0.0
        
        # Variable de salida
        self.u = 0.0

        # Timer para ejecutar el control a intervalos regulares
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Parameter Callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Se inicializa apagado el controlador
        self.control_running = False

        # Se crea crea nuestro servicio
        self.srv = self.create_service(SwitchCtrl, 'EnableCtrl_ROSa',
            self.control_service_callback)
        
    def setpoint_callback(self, msg):
        self.set_point = msg.data

    def output_callback(self, msg):
        self.motor_output = msg.data

    def control_loop(self):
        # En caso de estar apagado el controlador no procesa nada
        if not self.control_running:
            # Manda nuestro setpoint directo a nuestro motor
            msg = Float32()
            msg.data = self.set_point
            self.publisher.publish(msg)
            return
            
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

    def parameters_callback(self, params):
        for param in params:
            #system gain parameter check
            if param.name == "kp":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kp! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kp cannot be negative")
                else:
                    self.kp = param.value  # Update internal variable
                    self.get_logger().info(f"kp updated to {self.kp}")
            elif param.name == "ki":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid ki! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="ki cannot be negative")
                else:
                    self.ki = param.value  # Update internal variable
                    self.get_logger().info(f"ki updated to {self.ki}")
            elif param.name == "kd":
                #check if it is negative
                if (param.value < 0.0):
                    self.get_logger().warn("Invalid kd! It just cannot be negative.")
                    return SetParametersResult(successful=False, reason="kd cannot be negative")
                else:
                    self.kd = param.value  # Update internal variable
                    self.get_logger().info(f"kd updated to {self.kd}")
        return SetParametersResult(successful=True)
    
    def control_service_callback(self, request, response):
        #Al cambiar el valor enable del Servicio se manda un mensaje
        if request.enable:
            self.control_running = True
            self.get_logger().info(" Controler Started")
            response.success = True
            response.message = "Controler started Succesfully"
        #En caso contrario tambien, esto con fin de debuggear
        else:
            self.control_running = False
            self.get_logger().info(" Controler stopped")
            response.success = True
            response.message = "Controler stopped succesfully"
        return response

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
