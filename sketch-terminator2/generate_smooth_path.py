#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String


class GenerateSmoothPath(Node):
    def __init__(self):
        super().__init__('generate_smooth_path')

        # --- PARAMETRI GIBANJA ROBOTA ---
        self.TARGET_SPEED = 0.1       # Brzina robota (0.1 m/s = 10 cm/s)
        self.TIMER_PERIOD = 0.04      # Frekvencija slanja na 'trenutna_tocka' (25 Hz)
        self.STEP_DISTANCE = self.TARGET_SPEED * self.TIMER_PERIOD  # Korak pomaka (4 mm)

        self.initial_tcp = None       # Sluša se samo jednom u životu čvora
        self.is_busy = False          # Zastavica koja označava izvršava li se trenutno gibanje

        # Lista svih ključnih točaka i praćenje trenutnog stanja
        self.execution_queue = []
        self.current_target_point = None
        self.virtual_pose = None
        self.last_drawing_point = [0.0, 0.0, 0.0]

        # Parametri za logiku čekanja (pauze)
        self.is_waiting = False
        self.wait_counter = 0
        self.WAIT_STEPS = int(1.0 / self.TIMER_PERIOD) # 1.0 sekunda = 25 koraka

        # --- SUBSCRIBERI I PUBLISHER ---
        # Sluša `/marker_end_point` samo na samom početku
        self.sub_initial_pose = self.create_subscription(
            Point, 
            'marker_end_point', 
            self.initial_pose_callback, 
            10
        )
        
        # Sluša putanju koju planer šalje (može primiti beskonačno mnogo zahtjeva)
        self.sub_planned_path = self.create_subscription(
            String, 
            '/planning/path', 
            self.planned_path_callback, 
            10
        )

        self.pub_current_point = self.create_publisher(Point, 'trenutna_tocka', 10)

        # Glavni navigacijski tajmer (25 Hz)
        self.move_timer = self.create_timer(self.TIMER_PERIOD, self.navigation_loop)

        self.get_logger().info("Čvor generate_smooth_path (VIŠEKRATNI MOD) pokrenut. Čekam prvu poziciju...")

    def initial_pose_callback(self, msg: Point):
        # Ovo se izvršava samo jednom u cijelom radu sustava
        if self.initial_tcp is None:
            self.initial_tcp = [msg.x, msg.y, msg.z]
            self.virtual_pose = [msg.x, msg.y, msg.z]
            self.get_logger().info(f"Ulovljena prva početna pozicija robota: X={msg.x:.4f}, Y={msg.y:.4f}, Z={msg.z:.4f}")
            # Isključujemo pretplatu jer nam pozicija s kinematike više nikada neće trebati
            self.destroy_subscription(self.sub_initial_pose)

    def planned_path_callback(self, msg: String):
        # Ako još ne znamo gdje je robot počeo, ignoriraj zahtjev
        if self.virtual_pose is None:
            self.get_logger().warn("Primljena putanja, ali početna pozicija robota još nije poznata!")
            return
        
        # Ako robot već izvršava neku putanju, ignoriraj novi zahtjev dok ne završi
        if self.is_busy:
            self.get_logger().warn("Robot je trenutno zauzet crtanjem! Ignoriram novi zahtjev.")
            return

        try:
            data = json.loads(msg.data)
            path_json = data.get("path", [])
        except json.JSONDecodeError:
            self.get_logger().error("Greška pri parsiranju JSON-a s /planning/path!")
            return

        if not path_json:
            self.get_logger().warn("Primljena putanja je prazna!")
            return

        planned_points = [[float(pt['x']), float(pt['y']), 0.0] for pt in path_json]
        self.get_logger().info(f"Primljen NOVI zahtjev za crtanje ({len(planned_points)} točaka). Generiram sekvencu...")

        # --- SLAGANJE TOČAKA PO KORACIMA (Oslanja se na trenutnu virtualnu poziciju) ---
        self.execution_queue = []
        
        # Uzimamo trenutnu poziciju (gdje god robot bio u tom trenutku, na početku ili na kraju prošle rute)
        start_x, start_y, start_z = self.virtual_pose
        
        # 1. Podigni se ravno gore s trenutne pozicije na Z = 0.09m
        self.execution_queue.append([start_x, start_y, 0.09])
        
        # 2. Dođi do prve sigurne točke X = 0.2, Y = 0, Z = 0.09m
        self.execution_queue.append([0.15, 0.0, 0.09])
        
        # 3. Od tamo dođi iznad prve točke iz path planninga (Z = 0.09m)
        self.execution_queue.append([planned_points[0][0], planned_points[0][1], 0.09])
        
        # 4. Spusti se na pod (Z = 0.0) na prvu točku crtanja
        self.execution_queue.append([planned_points[0][0], planned_points[0][1], 0.0])
        
        # 5. Prođi kroz sve točke koje je generirao planer
        for pt in planned_points[1:]:
            self.execution_queue.append(pt)

        # Bilježimo zadnju točku crtanja radi aktivacije pauze
        self.last_drawing_point = planned_points[-1]

        # 6. Nakon čekanja (koje se obrađuje u petlji), podigni se na Z = 0.09m iznad zadnje točke
        self.execution_queue.append([planned_points[-1][0], planned_points[-1][1], 0.09])
        
        # 7. Dođi opet u točku X = 0.2, Y = 0, Z = 0.09m
        self.execution_queue.append([0.15, 0.0, 0.09])
        
        # 8. Dođi do konačne sigurne točke X = 0.0, Y = 0.2, Z = 0.09m
        self.execution_queue.append([0.0, 0.15, 0.09])

        # Postavi prvu ciljnu točku i zaključaj čvor za nove zahtjeve dok crtanje traje
        self.current_target_point = self.execution_queue.pop(0)
        self.is_busy = True
        self.get_logger().info("Gibanje pokrenuto! Šaljem glatke točke...")

    def navigation_loop(self):
        # Ako nema aktivnog zadatka, tajmer miruje i čeka novu poruku s planera
        if not self.is_busy or self.virtual_pose is None or self.current_target_point is None:
            return

        # --- LOGIKA ČEKANJA (PAUZA OD 1 SEKUNDE NA ZADNJOJ TOČKI CRTANJA) ---
        if self.is_waiting:
            self.wait_counter += 1
            if self.wait_counter >= self.WAIT_STEPS:
                self.is_waiting = False
                self.wait_counter = 0
                self.get_logger().info("Pauza završena. Nastavljam prema sigurnim točkama...")
                if len(self.execution_queue) > 0:
                    self.current_target_point = self.execution_queue.pop(0)
                else:
                    self.current_target_point = None
            return

        curr_x, curr_y, curr_z = self.virtual_pose
        targ_x, targ_y, targ_z = self.current_target_point

        dx = targ_x - curr_x
        dy = targ_y - curr_y
        dz = targ_z - curr_z
        distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)

        # Ako smo stigli blizu trenutne pod-točke (tolerancija 3 mm)
        if distance_to_target < 0.003:
            # Provjera za pauzu na kraju crtanja
            if abs(curr_x - self.last_drawing_point[0]) < 0.005 and abs(curr_y - self.last_drawing_point[1]) < 0.005 and abs(curr_z - 0.0) < 0.005:
                self.is_waiting = True
                self.get_logger().info("Robot na kraju putanje. Čekam 1 sekundu...")
                return

            # Uzmi sljedeću točku iz reda
            if len(self.execution_queue) > 0:
                self.current_target_point = self.execution_queue.pop(0)
                targ_x, targ_y, targ_z = self.current_target_point
                dx = targ_x - curr_x
                dy = targ_y - curr_y
                dz = targ_z - curr_z
                distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)
            else:
                # KRAJ CJELOKUPNOG GIBANJA ZA OVAJ ZAHTJEV
                self.get_logger().info("Gibanje završeno! Robot je na (0.0, 0.2, 0.09). Spreman za NOVI zahtjev s planera...")
                self.current_target_point = None
                self.is_busy = False # Otključavamo čvor za sljedeće pokretanje!
                return

        # --- INTERPOLACIJA OTVORENE PETLJE (Konstantna brzina 0.1 m/s) ---
        if distance_to_target > self.STEP_DISTANCE:
            scale = self.STEP_DISTANCE / distance_to_target
            self.virtual_pose[0] += dx * scale
            self.virtual_pose[1] += dy * scale
            self.virtual_pose[2] += dz * scale
        else:
            self.virtual_pose = [targ_x, targ_y, targ_z]

        # Slanje trenutne izračunate točke na temu 'trenutna_tocka'
        point_msg = Point()
        point_msg.x = self.virtual_pose[0]
        point_msg.y = self.virtual_pose[1]
        point_msg.z = self.virtual_pose[2]

        self.pub_current_point.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GenerateSmoothPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()