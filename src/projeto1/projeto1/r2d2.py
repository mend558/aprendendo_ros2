import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

import numpy as np
import matplotlib.pyplot as plt
from numpy import random


class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10,
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan',
                                 self.listener_callback_laser, qos_profile)

        self.get_logger().debug('Definindo o subscriber do laser: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom',
                                 self.listener_callback_odom, qos_profile)

        self.get_logger().debug(
            'Definindo o publisher de controle do robo: "/cmd_Vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            'Definindo buffer, listener e timer para acessar as TFs.')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def on_timer(self):
        try:
            self.tf_right = self.tf_buffer.lookup_transform(
                "right_center_wheel", "right_leg_base", rclpy.time.Time())

            _, _, self.right_yaw = tf_transformations.euler_from_quaternion([
                self.tf_right.transform.rotation.x,
                self.tf_right.transform.rotation.y,
                self.tf_right.transform.rotation.z,
                self.tf_right.transform.rotation.w
            ])

            self.get_logger().debug(
                f'yaw right_leg_base to right_center_wheel: {self.right_yaw}')

        except TransformException as ex:
            self.get_logger().debug(
                f'Could not transform right_leg_base to right_center_wheel: {ex}'
            )

        try:
            self.tf_left = self.tf_buffer.lookup_transform(
                "left_center_wheel", "left_leg_base", rclpy.time.Time())

            _, _, self.left_yaw = tf_transformations.euler_from_quaternion([
                self.tf_left.transform.rotation.x,
                self.tf_left.transform.rotation.y,
                self.tf_left.transform.rotation.z,
                self.tf_left.transform.rotation.w
            ])

            self.get_logger().debug(
                f'yaw left_leg_base to left_center_wheel: {self.left_yaw}')

        except TransformException as ex:
            self.get_logger().debug(
                f'Could not transform left_leg_base to left_center_wheel: {ex}'
            )

    # [Gaussiana]
    def gaussian(self, x, mean, sigma):
        return (1 / (sigma * np.sqrt(2 * np.pi))) * np.exp(-((x - mean)**2) /
                                                           (2 * sigma**2))

    # [Gaussiana]

    def run(self):

        self.get_logger().debug(
            'Executando uma iteração do loop de processamento de mensagens.')
        rclpy.spin_once(self)

        self.get_logger().debug('Definindo mensagens de controde do robô.')
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0),
                                    angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0),
                           angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.get_logger().info('Ordenando o robô: "ir para a frente"')
        self.pub_cmd_vel.publish(self.ir_para_frente)
        rclpy.spin_once(self)

        # [Valores]
        self.media = 0
        self.sigma_movimento = 0.006
        self.sigma_lidar = 0.175  # meters
        self.porta = 0
        self.mapa = [1, 4, 7]
        self.achouPorta = 0  # Variável que verifica se já está dentro do espaço de uma porta para não repetir a correção.
        # [Valores]

        # [Inicializar o gráfico]
        x = np.linspace(-1, 8,
                        500)  # cria um vetor x de 500 valores entre -4.5 e 7
        y = np.zeros(500)  # cria um vetor y de 500 valores zeros
        y2 = np.zeros(500)
        y3 = np.zeros(500)
        fig, ax = plt.subplots()
        # [Inicializar o gráfico]

        self.get_logger().info('Entrando no loop princial do nó.')
        while (rclpy.ok):
            rclpy.spin_once(self)
            # [Caminhar a média]
            self.media = self.media + 0.05 #+ (random.randint()/1000)  
            # [Caminhar a média]

            # [Aumentar o sigma de movimento]
            self.sigma_movimento = self.sigma_movimento + 0.006
            # [Aumentar o sigma de movimento]

            # [Plotar o gráfico da gaussiana]
            for i in range(len(x)):
                y[i] = self.gaussian(x[i], self.media, self.sigma_movimento)

            ax.clear()
            ax.set_ylim(0, 4)
            ax.plot(x, y, color="b")
            plt.pause(0.1)
            # [Plotar o gráfico da gaussiana]

            self.get_logger().debug(
                'Atualizando as distancias lidas pelo laser.')
            self.distancia_direita = max((self.laser[1:9]))  # -90 a -10 graus
            self.get_logger().info(
                f'distancia_direita: {self.distancia_direita}')

            self.distancia_frente = min(
                (self.laser[80:100]))  # -10 a  10 graus
            self.get_logger().debug(
                f'distancia_frente: {self.distancia_frente}')

            self.distancia_esquerda = max(
                (self.laser[170:179]))  #  10 a  90 graus
            self.get_logger().info(
                f'distancia_esquerda: {self.distancia_esquerda}')

            self.get_logger().info(f'posicao x: {self.media}')

            # [Correção]
            if (self.distancia_direita > 8 and self.distancia_esquerda > 8 and self.achouPorta == 0):

                # [Atualizar a média e sigma]
                
                # calcular a nova média
                media_nova = (self.mapa[self.porta] * self.sigma_movimento
                              + self.media * self.sigma_lidar) / (
                                  self.sigma_movimento + self.sigma_lidar)
                sigma_novo = 1 / (1 / self.sigma_movimento +
                                  1 / self.sigma_lidar)
                self.media = media_nova
                self.sigma_movimento = sigma_novo

                for i in range(len(x)): 
                    y2[i] = self.gaussian(x[i], self.mapa[self.porta], 
                                              self.sigma_lidar)
                    #ax.plot(x, y2, color="r")
                    # plota em vermelho “r” a gaussiana da leitura do laser com relação à porta

                for i in range(len(x)): 
                    y3[i] = self.gaussian(x[i], media_nova, sigma_novo)
                    #ax.plot(x, y3, color="g") 
                    # plota em verde “g” a gaussiana nova após interpolação das duas gaussianas.

                #plt.pause(1)

                self.achouPorta = 1
                    
                # [Aumentar a contagem de porta]
                if self.porta == 0: self.porta = 1
                elif self.porta == 1: self.porta = 2
                # [Aumentar a contagem de porta]

                # [Atualizar a média e sigma]
            # [Correção]
            
            # [Caso já tenha passado por uma porta, resetar a variavel achou porta]
            elif(self.distancia_esquerda < 5 and self.distancia_direita < 5 and 
                 self.achouPorta == 1): 
                self.achouPorta = 0
            # [Caso já tenha passado por uma porta, resetar a variavel achou porta]

                
            self.get_logger().debug("Distância para o obstáculo" +
                                    str(self.distancia_frente))
            if (self.distancia_frente < 1.5):
                self.get_logger().info('Obstáculo detectado.')
                break

        self.get_logger().info('Ordenando o robô: "parar"')
        self.pub_cmd_vel.publish(self.parar)
        rclpy.spin_once(self)

    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()