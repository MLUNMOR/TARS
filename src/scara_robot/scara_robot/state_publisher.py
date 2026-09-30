from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class PickAndPlacePublisher(Node):

    def __init__(self):
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.get_logger().info("Iniciando secuencia Pick & Place para el robot SCARA...")

        # Estado actual de las articulaciones
        self.arm1_angle = 0.0
        self.arm2_angle = 0.0
        self.z_pos = 0.0
        self.gripper_pos = 0.015  # Empezamos con la pinza abierta (1.5 cm)

        # Objetivos de posición para PICK (recoger) y PLACE (soltar)
        # Posición Pick: Brazo1 a 45º (0.78 rad), Brazo2 a 30º (0.52 rad)
        self.pick_arm1 = 0.78
        self.pick_arm2 = 0.52

        # Posición Place: Brazo1 a -60º (-1.04 rad), Brazo2 a -45º (-0.78 rad)
        self.place_arm1 = -1.04
        self.place_arm2 = -0.78

        # Máquina de estados
        self.state = 0
        self.wait_ticks = 0  # Temporizador simple para pausas

        # Bucle de control a 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.update_sequence)

    def update_sequence(self):
        # Velocidades de movimiento por ciclo
        deg_speed = 0.02   # Velocidad angular
        z_speed = 0.003    # Velocidad lineal Z
        grip_speed = 0.001 # Velocidad pinza

        # --- MÁQUINA DE ESTADOS PICK & PLACE ---
        if self.state == 0:
            # FASE 0: Mover brazos sobre la zona de PICK
            self.arm1_angle = self.move_towards(self.arm1_angle, self.pick_arm1, deg_speed)
            self.arm2_angle = self.move_towards(self.arm2_angle, self.pick_arm2, deg_speed)

            # Cuando ambos brazos llegan al objetivo, pasamos a bajar Z
            if self.arm1_angle == self.pick_arm1 and self.arm2_angle == self.pick_arm2:
                self.state = 1

        elif self.state == 1:
            # FASE 1: Bajar eje Z hasta la pieza (-0.12 m)
            self.z_pos = self.move_towards(self.z_pos, -0.12, z_speed)
            if self.z_pos == -0.12:
                self.state = 2

        elif self.state == 2:
            # FASE 2: Cerrar pinza para agarrar (0.0 m)
            self.gripper_pos = self.move_towards(self.gripper_pos, 0.0, grip_speed)
            if self.gripper_pos == 0.0:
                self.state = 3

        elif self.state == 3:
            # FASE 3: Subir eje Z con la pieza
            self.z_pos = self.move_towards(self.z_pos, 0.0, z_speed)
            if self.z_pos == 0.0:
                self.state = 4

        elif self.state == 4:
            # FASE 4: Mover brazos a la zona de PLACE
            self.arm1_angle = self.move_towards(self.arm1_angle, self.place_arm1, deg_speed)
            self.arm2_angle = self.move_towards(self.arm2_angle, self.place_arm2, deg_speed)
            if self.arm1_angle == self.place_arm1 and self.arm2_angle == self.place_arm2:
                self.state = 5

        elif self.state == 5:
            # FASE 5: Bajar Z en la zona de entrega
            self.z_pos = self.move_towards(self.z_pos, -0.12, z_speed)
            if self.z_pos == -0.12:
                self.state = 6

        elif self.state == 6:
            # FASE 6: Abrir pinza para soltar la pieza (0.015 m)
            self.gripper_pos = self.move_towards(self.gripper_pos, 0.015, grip_speed)
            if self.gripper_pos == 0.015:
                self.state = 7

        elif self.state == 7:
            # FASE 7: Subir Z y reiniciar el bucle
            self.z_pos = self.move_towards(self.z_pos, 0.0, z_speed)
            if self.z_pos == 0.0:
                self.state = 0  # Volver a empezar el ciclo

        # --- PUBLICACIÓN DE MENSAJES ---
        now = self.get_clock().now()

        # Publicar JointState
        joint_state = JointState()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = [
            'arm1_joint',
            'arm2_joint',
            'z_joint',
            'left_finger_joint',
            'right_finger_joint'
        ]
        joint_state.position = [
            self.arm1_angle,
            self.arm2_angle,
            self.z_pos,
            self.gripper_pos,
            -self.gripper_pos
        ]
        self.joint_pub.publish(joint_state)

        # Publicar TF (Base fija en el origen)
        odom_trans = TransformStamped()
        odom_trans.header.stamp = now.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_footprint'
        odom_trans.transform.translation.x = 0.0
        odom_trans.transform.translation.y = 0.0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = euler_to_quaternion(0, 0, 0)

        self.broadcaster.sendTransform(odom_trans)

    # Función auxiliar para interpolar de un valor actual a un objetivo paso a paso
    def move_towards(self, current, target, step):
        if abs(current - target) <= step:
            return target
        return current + step if current < target else current - step


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlacePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()