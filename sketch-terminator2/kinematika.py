#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState  # Dodano za čitanje pozicija motora

def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

class Kinematika(Node):
    def __init__(self):
        super().__init__('kinematika')

        # Publisher za izračunatu poziciju vrha robota (TCP)
        self.pub_end_point = self.create_publisher(Point, 'marker_end_point', 10)

        # Pretplate se na stvarni topic s motorima umjesto umjetnog Point topica
        self.joint_states_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        self.get_logger().info("Čvor kinematike uspješno pokrenut. Slušam /joint_states...")

    def joint_state_callback(self, msg: JointState):
        # Inicijalizacija varijabli za kutove
        beta_msg = None
        gama_msg = None
        alpha_msg = None

        # Prolazimo kroz sve zglobove u poruci i mapiramo ih prema tvom zahtjevu:
        # joint1 -> beta, joint2 -> gama, joint3 -> alpha
        for i, name in enumerate(msg.name):
            if name == 'joint1':
                beta_msg = msg.position[i]
            elif name == 'joint2':
                gama_msg = msg.position[i]
            elif name == 'joint3':
                alpha_msg = msg.position[i]

        # Ako poruka ne sadrži sva tri zgloba, preskačemo trenutni korak proračuna
        if beta_msg is None or gama_msg is None or alpha_msg is None:
            self.get_logger().warn("Nisu primljeni položaji za sva 3 zgloba (joint1, joint2, joint3)!")
            return

        # --- GEOMETRIJSKI PRORAČUN (Zadržana tvoja izvorna logika) ---
        
        # 1. Vrati kutove u izvorne geometrijske kutove iz IK
        alpha = alpha_msg + math.pi / 2
        
        # Iz IK znamo: beta_msg = math.pi/2 - b  =>  b = math.pi/2 - beta_msg
        b = math.pi / 2 - beta_msg
        
        # Iz IK znamo: gama_msg = math.pi - g1   =>  g1 = math.pi - gama_msg
        g1 = math.pi - gama_msg

        # Duljine linkova
        L1 = 0.2
        L2 = 0.2531
        H_base = 0.09

        # 2. Trigonometrija u ravnini ruke (R-Z ravnina)
        # Poučak o kosinusu za pronalaženje duljine r1 (udaljenost od zgloba beta do vrha)
        r1 = math.sqrt(L1**2 + L2**2 - 2 * L1 * L2 * math.cos(g1))

        # Traženje kuta b2 (kut između L1 i r1)
        cos_b2 = (L1**2 + r1**2 - L2**2) / (2 * L1 * r1)
        cos_b2 = max(-1.0, min(1.0, cos_b2))
        b2 = math.acos(cos_b2)

        # Budući da je b = math.pi - b1 - b2, imamo:
        b1 = math.pi - b - b2

        # Razlaganje na horizontalnu (r) i vertikalnu (h1) komponentu
        r = r1 * math.sin(b1)
        h1 = r1 * math.cos(b1)

        # Visina vrha (z) u odnosu na bazu
        mz = H_base - h1

        # 3. Projekcija radijusa r na X i Y osi pomoću kuta okretanja baze (alpha)
        mx = r * math.cos(alpha)
        my = r * math.sin(alpha)

        # Ako koristiš pomake baze iz IK, otkomentiraj linije ispod:
        # mx += 0.05195
        # my += 0.175

        # 4. Slanje rezultata
        vrh = Point()
        vrh.x = mx
        vrh.y = my
        vrh.z = mz
        
        self.pub_end_point.publish(vrh)

def main(args=None):
    rclpy.init(args=args)
    node = Kinematika()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()