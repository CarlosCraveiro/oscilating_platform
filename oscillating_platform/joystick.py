import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node

# Interfaces do Crazyswarm
from crazyflie_interfaces.msg import Position, FullState
from crazyflie_interfaces.srv import GoTo, NotifySetpointsStop, Land # <-- Land importado
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
        self.pos_publisher_ = self.create_publisher(Position, '/crazyflie/cmd_position', 10)
        self.full_state_pub = self.create_publisher(FullState, '/crazyflie/cmd_full_state', 10)
        self.z_mocap_pub = self.create_publisher(Float64, '/platform_prediction/z_mocap', 10)
        
        self.goto_client = self.create_client(GoTo, '/crazyflie/go_to')
        self.notify_client = self.create_client(NotifySetpointsStop, '/crazyflie/notify_setpoints_stop')
        self.land_client = self.create_client(Land, '/crazyflie/land') # <-- Cliente de Pouso (Corte de Motor)
        
        # ==========================================
        # INSCRIÇÕES (SENSORES E PREVISÕES)
        # ==========================================
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pose_subscriber = self.create_subscription(
            NamedPoseArray, '/poses', self.pose_callback, qos_profile_sensor_data)
            
        self.sub_pred_z = self.create_subscription(Float64, '/platform_prediction/z_future', self.pred_z_callback, 10)
        self.sub_pred_z_dot = self.create_subscription(Float64, '/platform_prediction/z_dot_future', self.pred_z_dot_callback, 10)
         
        # ==========================================
        # ESTADOS DO DRONE E PLATAFORMA
        # ==========================================
        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.3      
        self.plat_x = 0.0
        self.plat_y = 0.0
        self.plat_z = 0.0
        
        self.predicted_z = None
        self.predicted_z_dot = 0.0 
        
        # Máquina de Estados e Timers Seguros
        self.current_mode = 'IDLE'       
        self.was_in_low_level = False    
        self.last_buttons = [0] * 15 # Array longo para suportar todos os índices de joystick
        
        # Timers separados para evitar Race Conditions em transições em cadeia
        self.action_timer = None     
        self.sequence_timer = None
        
        # ==========================================
        # PARÂMETROS DE VOO (OITO)
        # ==========================================
        self.eight_flight_z = 0.3
        self.t = 0.0
        self.center_x = 0.0
        self.center_y = 0.0
        self.size_x = 0.6
        self.size_y = 0.6
        self.speed = 0.5
        
        # ==========================================
        # PARÂMETROS DE POUSO DINÂMICO
        # ==========================================
        self.landing_speed_m_s = 0.2       # Velocidade de descida (0.2 m/s)
        self.landing_safety_margin_sec = 1.5 # Tempo extra forçando o drone contra o solo
        self.current_landing_offset = 0.3  # Começa a 30cm
        self.min_landing_offset = -0.3
        self.landing_time_elapsed = 0.0
        self.expected_landing_time = 0.0
        
        # Timer multifunção (20Hz)
        self.timer_period = 0.05
        self.loop_timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info('Joystick Pronto: [0:Oito | 1:Origem | 2:Ir Plataforma | 3:Surfar | 12:Pousar]')

    # ------------------------------------------
    # CALLBACKS DE ATUALIZAÇÃO
    # ------------------------------------------
    def pose_callback(self, msg):
        for item in msg.poses:
            if 'crazyflie' in item.name:
                self.drone_x = item.pose.position.x
                self.drone_y = item.pose.position.y
                self.drone_z = item.pose.position.z
            elif 'osci_plat' in item.name:
                self.plat_x = item.pose.position.x
                self.plat_y = item.pose.position.y
                self.plat_z = item.pose.position.z
                self.z_mocap_pub.publish(Float64(data=self.plat_z))

    def pred_z_callback(self, msg):
        self.predicted_z = msg.data

    def pred_z_dot_callback(self, msg):
        self.predicted_z_dot = msg.data

    # ------------------------------------------
    # CALLBACK DO JOYSTICK (MÁQUINA DE ESTADOS)
    # ------------------------------------------
    def joy_callback(self, msg):
        if len(msg.buttons) == 0:
            return

        # Prevenção segura contra controles com menos botões
        btn_0 = msg.buttons[0] if len(msg.buttons) > 0 else 0
        btn_1 = msg.buttons[1] if len(msg.buttons) > 1 else 0
        btn_2 = msg.buttons[2] if len(msg.buttons) > 2 else 0
        btn_3 = msg.buttons[3] if len(msg.buttons) > 3 else 0
        btn_12 = msg.buttons[12] if len(msg.buttons) > 12 else 0

        # --- BOTÃO 0: Figura 8 ---
        if btn_0 == 1 and self.last_buttons[0] == 0:
            if self.current_mode != 'EIGHT':
                self.get_logger().info(f'Iniciando Figura 8 na altura travada de {self.drone_z:.2f}m...')
                self.eight_flight_z = self.drone_z
                self.t = 0.0
                self.current_mode = 'EIGHT'
                self.was_in_low_level = True

        # --- BOTÃO 1: Retorno Seguro para a Origem ---
        elif btn_1 == 1 and self.last_buttons[1] == 0:
            self.get_logger().info(f'Voltando para X=0, Y=0 (mantendo Z={self.drone_z:.2f}m)...')
            self.current_mode = 'RETURNING_TO_ORIGIN'
            
            # 1. Vai para a Origem na altura atual instantânea
            # 2. Aguarda 3.5s
            # 3. Desce para 0.5m
            self.safe_transition_goto(
                0.0, 0.0, self.drone_z, duration_sec=3.0,
                next_action=self.step_origin_descend, next_delay=3.5
            )

        # --- BOTÃO 2: Aproximação Segura na Plataforma ---
        elif btn_2 == 1 and self.last_buttons[2] == 0:
            self.get_logger().info('Subindo verticalmente para 1m...')
            self.current_mode = 'GOING_TO_PLATFORM'
            self.safe_transition_goto(
                self.drone_x, self.drone_y, 1.0, duration_sec=2.0, 
                next_action=self.step_goto_platform, next_delay=2.5
            )

        # --- BOTÃO 3: Toggle de Compensação (Surfar) ---
        elif btn_3 == 1 and self.last_buttons[3] == 0:
            if self.current_mode not in ['COMPENSATING', 'DYNAMIC_LANDING']:
                if self.predicted_z is None:
                    self.get_logger().warn('Aguardando estimador! Não é possível compensar ainda.')
                else:
                    self.get_logger().info('Surfe Iniciado! Mantendo 30cm de distância da base.')
                    self.current_mode = 'COMPENSATING'
                    self.current_landing_offset = 0.3 
                    self.was_in_low_level = True
            else:
                self.get_logger().info('Surfe/Pouso Cancelado. Subindo para 1m sobre a plataforma.')
                self.current_mode = 'IDLE'
                self.safe_transition_goto(self.plat_x, self.plat_y, 1.0, duration_sec=2.0)

        # --- BOTÃO 12: Pouso Dinâmico Baseado em Tempo ---
        elif btn_12 == 1 and self.last_buttons[12] == 0:
            if self.current_mode == 'COMPENSATING':
                # Calcula o tempo total = (distancia / velocidade) + margem
                self.expected_landing_time = (self.current_landing_offset / self.landing_speed_m_s) + self.landing_safety_margin_sec
                self.landing_time_elapsed = 0.0
                
                self.get_logger().info(f'POUSO MANUAL Iniciado! Tempo estimado até o corte: {self.expected_landing_time:.2f}s')
                self.current_mode = 'DYNAMIC_LANDING'
            else:
                self.get_logger().warn('Negado: O drone precisa estar no modo SURFAR (Botão 3) primeiro.')

        for i in range(len(msg.buttons)):
            if i < len(self.last_buttons):
                self.last_buttons[i] = msg.buttons[i]

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
            
        elif self.current_mode in ['COMPENSATING', 'DYNAMIC_LANDING'] and self.predicted_z is not None:
            
            if self.current_mode == 'DYNAMIC_LANDING':
                self.landing_time_elapsed += self.timer_period
                
                # Desce a Z gradativamente (Velocidade constante independente de lags)
                self.current_landing_offset -= self.landing_speed_m_s * self.timer_period
                
                # Trava a matemática na altura da plataforma para evitar empurrar o chão forte demais
                if self.current_landing_offset < self.min_landing_offset:
                    self.current_landing_offset = self.min_landing_offset

                # Verifica se o tempo estourou (Pouso Concluído)
                if self.landing_time_elapsed >= self.expected_landing_time:
                    self.get_logger().info('Tempo esgotado! Drone apoiado. Cortando motores (Land Call).')
                    self.execute_motor_cutoff()
                    return # Interrompe a publicação instantaneamente
                    
            msg = FullState()
            msg.header.frame_id = "mocap"
            msg.pose.position.x = self.plat_x
            msg.pose.position.y = self.plat_y
            
            # Injeta a altura compensada da onda + o deslocamento gradual do pouso
            msg.pose.position.z = self.predicted_z + self.current_landing_offset 
            
            msg.pose.orientation.x = 0.0; msg.pose.orientation.y = 0.0; msg.pose.orientation.z = 0.0; msg.pose.orientation.w = 1.0 
            msg.twist.linear.x = 0.0; msg.twist.linear.y = 0.0; msg.twist.linear.z = self.predicted_z_dot 
            
            self.full_state_pub.publish(msg)

    # ------------------------------------------
    # SISTEMA SEGURO DE TRANSIÇÃO E SERVIÇOS
    # ------------------------------------------
    def stop_low_level_control(self):
        """Desliga o streaming e rearma o controle nativo do Firmware"""
        if self.was_in_low_level:
            if self.notify_client.wait_for_service(timeout_sec=1.0):
                self.notify_client.call_async(NotifySetpointsStop.Request())
            self.was_in_low_level = False

    def execute_motor_cutoff(self):
        """Finaliza o voo de forma graciosa sem disparar alarmes no Firmware"""
        self.stop_low_level_control()
        self.current_mode = 'IDLE'
        
        # Enviar um Land super rápido diz ao High-Level Commander: "Você pousou, pare os motores"
        if self.land_client.wait_for_service(timeout_sec=1.0):
            req = Land.Request()
            req.group_mask = 0
            req.height = 0.0 #self.plat_z + self.min_landing_offset 
            req.duration = Duration(sec=0, nanosec=50000000) # 0.1s 
            self.land_client.call_async(req)

    def safe_transition_goto(self, target_x, target_y, target_z, duration_sec, next_action=None, next_delay=0.0):
        """Bloqueia condições de corrida usando timers isolados e micro-delay"""
        self.stop_low_level_control()
        
        # Limpa callbacks de timers antigos que possam estar rodando
        if self.action_timer is not None:
            self.action_timer.cancel()
        if self.sequence_timer is not None:
            self.sequence_timer.cancel()

        def delayed_execution():
            if self.action_timer is not None:
                self.action_timer.cancel() 
                
            self.send_goto(target_x, target_y, target_z, duration_sec)
            
            if next_action is not None and next_delay > 0:
                self.sequence_timer = self.create_timer(next_delay, next_action)
                
        self.action_timer = self.create_timer(0.1, delayed_execution)

    def step_goto_platform(self):
        if self.sequence_timer is not None:
            self.sequence_timer.cancel()
        if self.current_mode == 'GOING_TO_PLATFORM':
            self.get_logger().info(f'Indo para cima do Alvo X:{self.plat_x:.2f}, Y:{self.plat_y:.2f}')
            self.send_goto(self.plat_x, self.plat_y, 1.0, duration_sec=3.0)
            self.current_mode = 'IDLE'

    def step_origin_descend(self):
        if self.sequence_timer is not None:
            self.sequence_timer.cancel()
        if self.current_mode == 'RETURNING_TO_ORIGIN':
            self.get_logger().info('Chegou na origem (X=0, Y=0). Descendo para Z=0.5m...')
            self.send_goto(0.0, 0.0, 0.5, duration_sec=2.0)
            self.current_mode = 'IDLE'

    def send_goto(self, x, y, z, duration_sec):
        if self.goto_client.wait_for_service(timeout_sec=1.0):
            req = GoTo.Request()
            req.group_mask = 0
            req.relative = False
            req.goal.x = float(x); req.goal.y = float(y); req.goal.z = float(z)
            req.yaw = 0.0
            req.duration = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            self.goto_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
