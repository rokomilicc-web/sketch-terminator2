#!/usr/bin/env python3
import json
import cv2
import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
from cv_bridge import CvBridge


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

        # Vizualni debugger
        self.bridge = CvBridge()
        self.latest_path = []
        self.latest_objects = []
        
        self.path_sub = self.create_subscription(String, '/planning/path', self.path_callback, 10)
        self.image_sub = self.create_subscription(Image, '/dbg_image', self.image_callback, 10)
        self.debug_image_pub = self.create_publisher(Image, '/planning/debug_image', 10)

    def pixel_to_robot_frame(self, u, v):
        """
        Pretvara piksel koordinate (u, v) u 3D koordinate (X, Y) u METRIMA
        unutar baze robota pomoću rješavanja presjeka zrake i Z=0 ravnine.
        Koristi standardnu OpenCV konvenciju (P_cam = R * P_world + t).
        """
        # 1. Smjer zrake u kameri (Z naprijed = 1.0 kako koristi OpenCV)
        X_c = (u - self.cx) / self.fx
        Y_c = (v - self.cy) / self.fy
        ray_cam = np.array([X_c, Y_c, 1.0], dtype=np.float64)

        # R_cam_to_robot iz YAML-a je zapravo R_world_to_cam (OpenCV format)
        # t_cam_to_robot iz YAML-a je t_world_to_cam (u milimetrima)
        R = self.R_cam_to_robot
        t = self.t_cam_to_robot

        # P_cam = R * P_world + t
        # s * ray_cam = X_w * R[:,0] + Y_w * R[:,1] + t
        # X_w * R[:,0] + Y_w * R[:,1] - s * ray_cam = -t
        M = np.column_stack((R[:, 0], R[:, 1], -ray_cam))
        
        try:
            solution = np.linalg.solve(M, -t)
            X_w_mm = solution[0]
            Y_w_mm = solution[1]
        except np.linalg.LinAlgError:
            raise RuntimeError("Ray is parallel to the table.")

        x_robot_m = X_w_mm / 1000.0
        y_robot_m = Y_w_mm / 1000.0

        return float(x_robot_m), float(y_robot_m)

    def robot_to_pixel_frame(self, x, y):
        """
        Pretvara 3D koordinate (X, Y) u METRIMA na bazi robota (Z=0) 
        nazad u (u, v) piksele na slici s kamere (Inverzna pinhole projekcija).
        """
        # Koordinate su u metrima, prebacujemo ih u milimetre (kako su zapisane u t_cam_to_robot)
        X_w_mm = x * 1000.0
        Y_w_mm = y * 1000.0
        
        P_world = np.array([X_w_mm, Y_w_mm, 0.0], dtype=np.float64)
        
        R = self.R_cam_to_robot
        t = self.t_cam_to_robot
        
        # P_cam = R * P_world + t
        P_cam = R.dot(P_world) + t
        
        if P_cam[2] <= 0:
            return 0, 0 # Točka je iza kamere
            
        # Pinhole model projekcije
        u = (P_cam[0] / P_cam[2]) * self.fx + self.cx
        v = (P_cam[1] / P_cam[2]) * self.fy + self.cy
        
        return int(u), int(v)

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
            self.latest_objects = objects_data
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
                "bbox_corners": bbox_corners_robot,
                "px": int(cx),
                "py": int(cy)
            })

        # Spremanje za vizualizaciju i slanje na positions_topic
        self.latest_objects = objects_data
        self.publish_object_positions(objects_data)

    def path_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.latest_path = data.get("path", [])
        except json.JSONDecodeError:
            self.get_logger().error("Could not decode path JSON")

    def image_callback(self, msg):
        try:
            # ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # --- CRTANJE ISHODIŠTA ROBOTA (x: 0.0, y: 0.0) ---
            ox, oy = self.robot_to_pixel_frame(0.0, 0.0)
            # Crveni marker za ishodište baze
            cv2.drawMarker(cv_image, (ox, oy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=30, thickness=3)
            cv2.putText(cv_image, "ORIGIN (0, 0)", (ox + 10, oy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # --- CRTANJE OSI KOORDINATNOG SUSTAVA (X=crvena, Y=zelena, dužina 10cm) ---
            xx, xy = self.robot_to_pixel_frame(0.1, 0.0)
            yx, yy = self.robot_to_pixel_frame(0.0, 0.1)
            
            # X os (crvena - BGR: 0, 0, 255)
            cv2.arrowedLine(cv_image, (ox, oy), (xx, xy), (0, 0, 255), 3, tipLength=0.2)
            cv2.putText(cv_image, "X", (xx + 5, xy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            
            # Y os (zelena - BGR: 0, 255, 0)
            cv2.arrowedLine(cv_image, (ox, oy), (yx, yy), (0, 255, 0), 3, tipLength=0.2)
            cv2.putText(cv_image, "Y", (yx + 5, yy + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # --- CRTANJE POČETNE TOČKE ROBOTA (x: 0.0, y: 0.15) ---
            rx, ry = self.robot_to_pixel_frame(0.0, 0.15)
            # Cyan marker "X" s tekstom
            cv2.drawMarker(cv_image, (rx, ry), (255, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
            cv2.putText(cv_image, "HOME (0.0, 0.15)", (rx + 10, ry - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

            # --- ISPIS KOORDINATA PREPOZNATIH OBJEKATA ---
            for obj in self.latest_objects:
                px = obj.get("px")
                py = obj.get("py")
                x = obj.get("x")
                y = obj.get("y")
                cls = obj.get("class")
                if px is not None and py is not None:
                    text = f"{cls}: ({x:.2f}, {y:.2f})"
                    # Crni obrub za bolju čitljivost teksta
                    cv2.putText(cv_image, text, (px - 40, py + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
                    # Bijeli tekst iznutra
                    cv2.putText(cv_image, text, (px - 40, py + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # --- CRTANJE PUTANJE AKO POSTOJI ---
            if self.latest_path:
                prev_pt = None
                for p in self.latest_path:
                    px, py = self.robot_to_pixel_frame(p["x"], p["y"])
                    
                    # Crtanje kruga za čvor putanje (Magenta)
                    cv2.circle(cv_image, (px, py), 6, (255, 0, 255), -1) 
                    
                    # Spajanje linijom s prethodnom točkom (Žuta)
                    if prev_pt is not None:
                        cv2.line(cv_image, prev_pt, (px, py), (0, 255, 255), 3)
                    
                    prev_pt = (px, py)

                # Oznaka Starta (Zeleno) i Cilja (Crveno) ako postoji putanja
                if len(self.latest_path) >= 2:
                    sx, sy = self.robot_to_pixel_frame(self.latest_path[0]["x"], self.latest_path[0]["y"])
                    cv2.circle(cv_image, (sx, sy), 10, (0, 255, 0), -1)
                    
                    gx, gy = self.robot_to_pixel_frame(self.latest_path[-1]["x"], self.latest_path[-1]["y"])
                    cv2.circle(cv_image, (gx, gy), 10, (0, 0, 255), -1)

            # OpenCV to ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            self.debug_image_pub.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error drawing path on image: {e}")

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