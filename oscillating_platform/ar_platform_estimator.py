import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import LogDataGeneric
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64
import numpy as np
import collections

# Importações do TF2 (A Mágica da Geometria)
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs 
from geometry_msgs.msg import PointStamped

# =====================================================================
# MATEMÁTICA PURA: Modelo Autorregressivo com RLS (Mantido Intacto)
# =====================================================================
class ARModelRLS:
    def __init__(self, order, forgetting_factor=0.99, initial_covariance=1000.0):
        self.p = order
        self.lam = forgetting_factor
        self.theta = np.zeros(self.p)
        self.P = np.eye(self.p) * initial_covariance 
        self.history = collections.deque(maxlen=self.p)

    def update(self, z_t):
        if len(self.history) < self.p:
            self.history.append(z_t)
            return False

        phi_t = np.array(list(self.history))[::-1]
        z_hat = np.dot(phi_t, self.theta)
        e_t = z_t - z_hat

        P_phi = np.dot(self.P, phi_t)
        den = self.lam + np.dot(phi_t.T, P_phi)
        K_t = P_phi / den

        self.theta = self.theta + K_t * e_t
        self.P = (1.0 / self.lam) * (self.P - np.outer(K_t, np.dot(phi_t, self.P)))
        self.history.append(z_t)
        return True

    def predict_future(self, n_steps):
        if len(self.history) < self.p:
            return []

        predictions = []
        current_phi = np.array(list(self.history))[::-1]

        for _ in range(n_steps):
            z_pred = np.dot(current_phi, self.theta)
            predictions.append(z_pred)
            current_phi = np.roll(current_phi, 1)
            current_phi[0] = z_pred

        return predictions

# =====================================================================
# LÓGICA ROS 2: Nó Consumidor de TF e Publicador de Previsão
# =====================================================================
class PlatformEstimatorNode(Node):
    def __init__(self):
        super().__init__('platform_ar_estimator')
        
        # --- PARÂMETROS ---
        self.freq_hz = 100.0          
        self.dt = 1.0 / self.freq_hz 
        self.N_predictions = 25     
        self.ar_order = 8
        self.alpha_filter = 0.2  # Ajuste entre 0.1 (muito suave, mais atraso) e 0.9 (pouco suave, rápido)
        self.filtered_z_plat = None

        # Frames (Ajuste para os nomes corretos do seu sistema Crazyswarm)
        self.world_frame = 'mocap'
        self.drone_frame = 'crazyflie' # Frame base do drone (base_link)
        # -------------------

        self.ar_model = ARModelRLS(order=self.ar_order)
        self.latest_z_plat = None
        self.drone_pos = {'x': 0.0, 'y': 0.0} # Atualizado via TF

        # Configuração do TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Inscrição (Apenas Laser, o Pose foi removido pois consumimos via TF)
        self.create_subscription(LogDataGeneric, '/crazyflie/range_finder_z', self.range_callback, 10)
        
        # Publicadores
        self.marker_pub = self.create_publisher(MarkerArray, '/platform_markers', 10)
        self.pred_pub = self.create_publisher(Float64, '/platform_prediction/z_future', 10)
        self.offset_pub = self.create_publisher(Float64, '/platform_prediction/time_offset', 10)
        
        # NOVO: Publicador da velocidade prevista (Feedforward)
        self.z_dot_pub = self.create_publisher(Float64, '/platform_prediction/z_dot_future', 10)
        self.z_real_pub = self.create_publisher(Float64, '/platform_prediction/z_real', 10)
        self.z_filtered_pub = self.create_publisher(Float64, '/platform_prediction/z_filtered', 10)

        # Timer
        self.create_timer(self.dt, self.timer_estimation_step)
        
        self.get_logger().info(f"AR Estimator (TF2) Iniciado: {self.freq_hz}Hz")

    def range_callback(self, msg):
        range_m = msg.values[0] / 1000.0 
        
        # Cria um ponto virtual no referencial local do drone (Laser apontando para baixo)
        point_in_drone = PointStamped()
        point_in_drone.header.frame_id = self.drone_frame
        point_in_drone.header.stamp = self.get_clock().now().to_msg()
        point_in_drone.point.x = 0.0
        point_in_drone.point.y = 0.0
        point_in_drone.point.z = -range_m # Distância lida no eixo Z negativo do drone

        try:
            # Busca a transformação matemática do momento exato (orientação e posição)
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, 
                self.drone_frame, 
                rclpy.time.Time()
            )
            
            # A mágica do TF: rotaciona e translada o ponto para o frame global
            point_in_mocap = tf2_geometry_msgs.do_transform_point(point_in_drone, transform)
            
            # Agora temos o Z real da plataforma, compensando Roll e Pitch
            self.latest_z_plat = point_in_mocap.point.z
            
            # Aproveitamos o TF para pegar X e Y para os marcadores do RViz
            self.drone_pos['x'] = transform.transform.translation.x
            self.drone_pos['y'] = transform.transform.translation.y
            
        except Exception as e:
            # Ignora erros iniciais enquanto a árvore de TF não estiver completamente publicada
            self.get_logger().warn(f"Aguardando arvore TF: {e}", throttle_duration_sec=2.0)

    def timer_estimation_step(self):
        
        if self.latest_z_plat is None:
            return

        if self.filtered_z_plat is None:
            self.filtered_z_plat = self.latest_z_plat
        else:
            self.filtered_z_plat = (self.alpha_filter * self.latest_z_plat) + \
                                   ((1.0 - self.alpha_filter) * self.filtered_z_plat)

        # 1. Atualiza o modelo RLS
        is_ready = self.ar_model.update(self.filtered_z_plat)

        if is_ready:
            # 2. Calcula as previsões
            future_zs = self.ar_model.predict_future(self.N_predictions)
            # Define qual passo usar (Ex: Índice 1 = 2 passos à frente = 200ms)
            target_idx = 3 
            
            target_prediction = future_zs[target_idx]
            time_offset = (target_idx + 1) * self.dt # +1 porque o índice 0 é o primeiro passo
            
            # Z Dot calculado com base no passo escolhido e o imediatamente anterior
            if target_idx > 0:
                z_dot = (future_zs[target_idx] - future_zs[target_idx - 1]) / self.dt
            else:
                # Se escolher o índice 0, compara com o valor filtrado atual
                z_dot = (future_zs[0] - self.filtered_z_plat) / self.dt
            
            # 4. Publica os tópicos
            self.pred_pub.publish(Float64(data=target_prediction))
            self.offset_pub.publish(Float64(data=time_offset))
            self.z_dot_pub.publish(Float64(data=z_dot)) # <-- Pronto para o Feedforward
            self.z_filtered_pub.publish(Float64(data=self.filtered_z_plat))
            self.z_real_pub.publish(Float64(data=self.latest_z_plat))
            
            # 5. Renderiza no RViz
            self.publish_rviz_markers(self.filtered_z_plat, future_zs)

    def publish_rviz_markers(self, current_z, future_zs):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        
        # Marcador Atual (Verde)
        current_marker = self.create_base_marker(0, stamp)
        current_marker.pose.position.z = current_z
        current_marker.color.r = 0.0; current_marker.color.g = 1.0; current_marker.color.a = 0.9
        marker_array.markers.append(current_marker)
        
        # Marcadores Futuros (Amarelos com "Fade Out")
        for i, pred_z in enumerate(future_zs):
            pred_marker = self.create_base_marker(i + 1, stamp)
            pred_marker.pose.position.z = pred_z
            pred_marker.pose.position.y = self.drone_pos['y'] + (i + 1) * 0.05 # Desloca p/ ver a onda
            
            pred_marker.scale.x = 0.2; pred_marker.scale.y = 0.2
            pred_marker.color.r = 1.0; pred_marker.color.g = 0.8; pred_marker.color.b = 0.0
            pred_marker.color.a = max(0.2, 0.8 - (i * (0.6 / len(future_zs)))) 
            
            marker_array.markers.append(pred_marker)

        self.marker_pub.publish(marker_array)

    def create_base_marker(self, m_id, stamp):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp
        marker.ns = "landing_platform"
        marker.id = m_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = self.drone_pos['x']
        marker.pose.position.y = self.drone_pos['y']
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.02
        marker.color.b = 0.0
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = PlatformEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
