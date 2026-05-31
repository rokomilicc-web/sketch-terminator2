#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import String
from yolo_msgs.msg import DetectionArray


class YoloWorkspaceProcessingNode(Node):
    def __init__(self):
        super().__init__('yolo_workspace_processing_node')

        # --- DEKLARACIJA ROS PARAMETARA (Učitavanje iz YAML-a) ---
        # Teme
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('positions_topic', '/vision/object_positions')

        # Intrinzični parametri
        self.declare_parameter('fx', 721.29907)
        self.declare_parameter('fy', 716.27563)
        self.declare_parameter('cx', 344.87706)
        self.declare_parameter('cy', 292.58597)

        # Ekstrinzični parametri (Deklariramo ih kao liste tipa float)
        self.declare_parameter('R_cam_to_robot', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('t_cam_to_robot', [0.0, 0.0, 0.0])

        # --- DOHVAĆANJE PARAMETARA U LOKALNE VARIJABLE ---
        yolo_topic = self.get_parameter('yolo_detections_topic').get_parameter_value().string_value
        positions_topic = self.get_parameter('positions_topic').get_parameter_value().string_value

        self.fx = self.get_parameter('fx').get_parameter_value().double_value
        self.fy = self.get_parameter('fy').get_parameter_value().double_value
        self.cx = self.get_parameter('cx').get_parameter_value().double_value
        self.cy = self.get_parameter('cy').get_parameter_value().double_value

        # Pretvaranje jednodimenzionalne liste od 9 elemenata natrag u 3x3 NumPy matricu rotacije
        r_list = self.get_parameter('R_cam_to_robot').get_parameter_value().double_array_value
        self.R_cam_to_robot = np.array(r_list, dtype=np.float64).reshape(3, 3)

        # Dohvaćanje vektora translacije (X, Y, Z) u mm
        t_list = self.get_parameter('t_cam_to_robot').get_parameter_value().double_array_value
        self.t_cam_to_robot = np.array(t_list, dtype=np.float64)

        self.get_logger().info("Successfully loaded camera parameters from configuration file!")
        self.get_logger().info(f"Loaded Z-height (camera distance): {self.t_cam_to_robot[2]:.2f} mm")

        # Pretplate i oglašavanja
        self.yolo_sub = self.create_subscription(DetectionArray, yolo_topic, self.yolo_callback, 10)
        self.positions_pub = self.create_publisher(String, positions_topic, 10)

    def pixel_to_robot_frame(self, u, v):
        """
        Pretvara piksel koordinate (u, v) u 3D koordinate (X, Y) u METRIMA
        unutar baze robota koristeći zraku projekcije, Z_robot = 0 i kalibracijski offset.
        """
        # 1. Smjer zrake u koordinatnom sustavu kamere (Pinhole model)
        X_c = (u - self.cx) / self.fx
        Y_c = (v - self.cy) / self.fy

        # Okretanje optičke osi kamere sukladno geometriji
        Z_c = -1.0
        ray_cam = np.array([X_c, Y_c, Z_c], dtype=np.float64)

        # 2. Transformacija smjera zrake u koordinatni sustav baze robota (samo rotacija R)
        ray_robot = self.R_cam_to_robot.dot(ray_cam)

        # 3. Računanje presjeka zrake s ravninom stola Z_robot = 0
        if ray_robot[2] == 0:
            raise RuntimeError("Ray is parallel to the robot base plane; division by zero.")

        s = -self.t_cam_to_robot[2] / ray_robot[2]

        x_robot_raw = self.t_cam_to_robot[0] + s * ray_robot[0]
        y_robot_raw = self.t_cam_to_robot[1] + s * ray_robot[1]

        # Invertiranje predznaka radi usklađivanja smjera matrice (izvorni korak)
        x_robot_inverted = -x_robot_raw
        y_robot_inverted = -y_robot_raw

        # --- PRIMJENA SUSTAVNOG POMAKA (U MILIMETRIMA) ---
        # Budući da su detektirane vrijednosti veće od stvarnih, oduzimamo offsete
        x_robot_corrected_mm = x_robot_inverted - 22.0
        y_robot_corrected_mm = y_robot_inverted - 9.0

        # --- PRETVORBA U METRE ---
        x_robot_m = x_robot_corrected_mm / 1000.0
        y_robot_m = y_robot_corrected_mm / 1000.0

        return float(x_robot_m), float(y_robot_m)

    def publish_object_positions(self, objects_data):
        msg_out = String()
        msg_out.data = json.dumps({
            "frame": "robot_base",
            "units": "m",
            "objects": objects_data
        })
        self.positions_pub.publish(msg_out)

    def yolo_callback(self, msg):
        objects_data = []

        if len(msg.detections) == 0:
            self.publish_object_positions(objects_data)
            return

        for detection in msg.detections:
            class_name = detection.class_name

            # YOLO bbox centar i dimenzije u pikselima
            cx = detection.bbox.center.position.x
            cy = detection.bbox.center.position.y
            w = detection.bbox.size.x
            h = detection.bbox.size.y

            try:
                # Računanje centra objekta u metrima
                x_robot, y_robot = self.pixel_to_robot_frame(cx, cy)

                # Definiranje 4 vrha pravokutnika u pikselima slike
                corners_pixels = [
                    ("top_left", cx - w / 2, cy - h / 2),
                    ("top_right", cx + w / 2, cy - h / 2),
                    ("bottom_right", cx + w / 2, cy + h / 2),
                    ("bottom_left", cx - w / 2, cy + h / 2)
                ]

                # Rječnik za preračunate 3D koordinate u metrima
                bbox_corners_robot = {}
                bbox_robot_points = []

                for name, px, py in corners_pixels:
                    bx_robot, by_robot = self.pixel_to_robot_frame(px, py)
                    
                    bbox_corners_robot[name] = {
                        "x": round(bx_robot, 4),
                        "y": round(by_robot, 4)
                    }
                    bbox_robot_points.append((bx_robot, by_robot))

                # Računanje min/max granica u metrima
                bbox_xs = [p[0] for p in bbox_robot_points]
                bbox_ys = [p[1] for p in bbox_robot_points]

                bbox_robot = {
                    "x_min": round(min(bbox_xs), 4),
                    "y_min": round(min(bbox_ys), 4),
                    "x_max": round(max(bbox_xs), 4),
                    "y_max": round(max(bbox_ys), 4),
                }

            except RuntimeError as e:
                self.get_logger().error(str(e))
                return

            # Dodavanje svih podataka u metrima u listu objekata
            objects_data.append({
                "class": class_name,
                "x": round(x_robot, 4),
                "y": round(y_robot, 4),
                "bbox": bbox_robot,
                "bbox_corners": bbox_corners_robot
            })

        # Slanje podataka na positions_topic za Path Planning nodu
        self.publish_object_positions(objects_data)


def main(args=None):
    rclpy.init(args=args)
    node = YoloWorkspaceProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()