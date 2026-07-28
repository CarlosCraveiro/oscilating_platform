import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

# Interfaces do Crazyswarm
from crazyflie_interfaces.msg import Position, FullState
from crazyflie_interfaces.srv import GoTo, NotifySetpointsStop
from sensor_msgs.msg import Joy
from motion_capture_tracking_interfaces.msg import NamedPoseArray
from std_msgs.msg import Float64
from builtin_interfaces.msg import Duration
import math

class JoystickCoordinatorNode(Node):
    def __init__(self):
        super().__init__('joystick_coordinator')
        
        # ==========================================
        # PUBLICADORES E CLIENTES DE SERVIÇO
        # ==========================================
        # Mantemos o cmd_position para a Figura 8 (movimento simples)
        self.pos_publisher_ = self.create_publisher(Position, '/crazyflie/cmd_position', 10)
        
        # NOVO: Publicador de Estado Completo para o Pouso/Compensação Feedforward
        self.full_state_pub = self.create_publisher(FullState, '/crazyflie/cmd_full_state', 10)
        
        self.goto_client = self.create_client(GoTo, '/crazyflie/go_to')
        self.notify_client = self.create_client(NotifySetpointsStop, '/crazyflie/notify_setpoints_stop')
        # NOVO: Publicador do Z real do MoCap para o PlotJuggler
        self.z_mocap_pub = self.create_publisher(Float64, '/platform_prediction/z_mocap', 10)
        
        # ==========================================
        # INSCRIÇÕES (SENSORES E PREVISÕES)
        # ==========================================
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pose_subscriber = self.create_subscription(
            NamedPoseArray, '/poses', self.pose_callback, qos_profile_sensor_data)
            
        # Inscrições nos tópicos do nosso Filtro AR
        self.sub_pred_z = self.create_subscription(Float64, '/platform_prediction/z_future', self.pred_z_callback, 10)
        self.sub_pred_z_dot = self.create_subscription(Float64, '/platform_prediction/z_dot_future', self.pred_z_dot_callback, 10)
         
        # ==========================================
        # ESTADOS DO DRONE, PLATAFORMA E PREVISÃO
        # ==========================================
        self.drone_z = 0.3      
        self.plat_x = 0.0
        self.plat_y = 0.0
        
        # Variáveis alimentadas pelo estimador AR (com fallbacks seguros)
        self.predicted_z = None
        self.predicted_z_dot = 0.0 
        
        # Máquina de Estados
        self.current_mode = 'IDLE'       
        self.was_in_low_level = False    
        self.last_buttons = [0, 0, 0, 0] 
        
        # ==========================================
        # VARIÁVEIS DA FIGURA 8
        # ==========================================
        self.eight_flight_z = 0.3
        self.t = 0.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.size_x = 0.6
        self.size_y = 0.6
        self.speed = 0.5
        
        # Timer multifunção (20Hz para comandos agressivos de baixo nível)
        self.timer_period = 0.05
        self.loop_timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info('Joystick [Pronto]: (0=Oito | 1=Origem | 2=Ir p/ Plataforma | 3=Surfar Plataforma)')

    # ------------------------------------------
    # CALLBACKS DE ATUALIZAÇÃO
    # ------------------------------------------
    def pose_callback(self, msg):
        for item in msg.poses:
            if 'crazyflie' in item.name:
                self.drone_z = item.pose.position.z
            elif 'osci_plat' in item.name:
                # Pegamos apenas X e Y do MoCap para a plataforma. Z virá do estimador.
                self.plat_x = item.pose.position.x
                self.plat_y = item.pose.position.y
                self.plat_z = item.pose.position.z
                self.z_mocap_pub.publish(Float64(data=self.plat_z))

    def pred_z_callback(self, msg):
        self.predicted_z = msg.data

    def pred_z_dot_callback(self, msg):
        self.predicted_z_dot = msg.data

    # ------------------------------------------
    # CALLBACK DO JOYSTICK (LÓGICA DOS BOTÕES)
    # ------------------------------------------
    def joy_callback(self, msg):
        if len(msg.buttons) < 4:
            return

        btn_0 = msg.buttons[0] 
        btn_1 = msg.buttons[1] 
        btn_2 = msg.buttons[2] 
        btn_3 = msg.buttons[3] 

        # --- BOTÃO 0: Figura 8 na altitude atual ---
        if btn_0 == 1 and self.last_buttons[0] == 0:
            if self.current_mode != 'EIGHT':
                self.get_logger().info(f'Iniciando Figura 8 na altura travada de {self.drone_z:.2f}m...')
                self.eight_flight_z = self.drone_z
                self.get_logger().info(f"z atual {self.drone_z}")
                self.t = 0.0
                self.current_mode = 'EIGHT'
                self.was_in_low_level = True

        # --- BOTÃO 1: Abortar e ir para Origem ---
        elif btn_1 == 1 and self.last_buttons[1] == 0:
            self.get_logger().info('Abortando... Retornando para a Origem (0, 0, 0.3m)')
            self.stop_low_level_control()
            self.current_mode = 'IDLE'
            self.send_goto(0.0, 0.0, 0.3, duration_sec=3)

        # --- BOTÃO 2: Aproximação Segura na Plataforma ---
        elif btn_2 == 1 and self.last_buttons[2] == 0:
            self.get_logger().info('Subindo para 1m e indo para cima da plataforma...')
            self.stop_low_level_control()
            self.current_mode = 'GOING_TO_PLATFORM'
            self.execute_platform_approach()

        # --- BOTÃO 3: Toggle de Compensação (Surfar) ---
        elif btn_3 == 1 and self.last_buttons[3] == 0:
            if self.current_mode != 'COMPENSATING':
                if self.predicted_z is None:
                    self.get_logger().warn('Aguardando dados do estimador AR! Não é possível compensar ainda.')
                else:
                    self.get_logger().info('Surfe Iniciado! Compensando Z com Feedforward (Z_dot)...')
                    self.current_mode = 'COMPENSATING'
                    self.was_in_low_level = True
            else:
                self.get_logger().info('Surfe Cancelado. Subindo para 1m sobre a plataforma.')
                self.stop_low_level_control()
                self.current_mode = 'IDLE'
                self.send_goto(self.plat_x, self.plat_y, 1.0, duration_sec=2)

        self.last_buttons = [btn_0, btn_1, btn_2, btn_3]

    # ------------------------------------------
    # LOOP DE CONTROLE (20Hz)
    # ------------------------------------------
    def control_loop(self):
        if self.current_mode == 'EIGHT':
            msg = Position()
            msg.header.frame_id = "mocap"
            msg.x = self.center_x + (self.size_x * math.sin(self.t * self.speed))
            msg.y = self.center_y + (self.size_y * math.sin(self.t * self.speed) * math.cos(self.t * self.speed))
            msg.z = self.eight_flight_z 
            msg.yaw = 0.0 
            self.pos_publisher_.publish(msg)
            self.t += self.timer_period
            
        elif self.current_mode == 'COMPENSATING' and self.predicted_z is not None:
            # A MÁGICA ACONTECE AQUI: Usando o FullState para injetar Velocidade e Posição
            msg = FullState()
            msg.header.frame_id = "mocap"
            
            # 1. Comando de Posição Alvo
            msg.pose.position.x = self.plat_x
            msg.pose.position.y = self.plat_y
            msg.pose.position.z = self.predicted_z + 0.3 # Mantém 30cm acima da previsão
            
            # Quaternion de Orientação (Yaw = 0 puro)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0 
            
            # 2. Comando Feedforward de Velocidade
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = self.predicted_z_dot # Acelera/Frea o drone junto com a onda
            
            # Publica o estado completo
            self.full_state_pub.publish(msg)

    # ------------------------------------------
    # SERVIÇOS AUXILIARES
    # ------------------------------------------
    def stop_low_level_control(self):
        """Desliga o streaming de baixo nível (impede conflitos e rearma o Watchdog)"""
        if self.was_in_low_level:
            if self.notify_client.wait_for_service(timeout_sec=1.0):
                self.notify_client.call_async(NotifySetpointsStop.Request())
            self.was_in_low_level = False

    def send_goto(self, x, y, z, duration_sec):
        if self.goto_client.wait_for_service(timeout_sec=1.0):
            req = GoTo.Request()
            req.group_mask = 0
            req.relative = False
            req.goal.x = float(x)
            req.goal.y = float(y)
            req.goal.z = float(z)
            req.yaw = 0.0
            req.duration = Duration(sec=duration_sec, nanosec=0) 
            self.goto_client.call_async(req)

    def execute_platform_approach(self):
        self.send_goto(0.0, 0.0, 1.0, duration_sec=3)
        self.create_timer(3.5, self.delayed_platform_goto)

    def delayed_platform_goto(self):
        if self.current_mode == 'GOING_TO_PLATFORM':
            self.get_logger().info(f'Indo para X:{self.plat_x:.2f}, Y:{self.plat_y:.2f} (Altura: 1m)')
            self.send_goto(self.plat_x, self.plat_y, 1.0, duration_sec=3)
            self.current_mode = 'IDLE' 
        return rclpy.timer.Timer.destroy

def main(args=None):
    rclpy.init(args=args)
    node = JoystickCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Joystick Encerrado.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
