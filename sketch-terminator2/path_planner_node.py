#!/usr/bin/env python3
import json
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_planner_node")

        self.declare_parameter("vision_topic", "/vision/object_positions")
        self.declare_parameter("request_topic", "/planning/request")
        self.declare_parameter("path_topic", "/planning/path")
        
        # Margina oko prepreke u metrima (povećano na 3 cm radi sigurnosti robota)
        self.declare_parameter("planning_margin_m", 0.03)

        vision_topic = self.get_parameter("vision_topic").value
        request_topic = self.get_parameter("request_topic").value
        path_topic = self.get_parameter("path_topic").value

        self.latest_vision_data = None

        self.vision_sub = self.create_subscription(String, vision_topic, self.vision_callback, 10)
        self.request_sub = self.create_subscription(String, request_topic, self.request_callback, 10)
        self.path_pub = self.create_publisher(String, path_topic, 10)

        self.get_logger().info("Poboljšani A* Visibility Graph planer pokrenut!")

    def vision_callback(self, ros_msg):
        try:
            self.latest_vision_data = json.loads(ros_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse vision JSON.")

    def request_callback(self, ros_msg):
        if self.latest_vision_data is None:
            self.get_logger().warn("Nema podataka s kamere još uvijek!")
            return

        try:
            request = json.loads(ros_msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Could not parse request JSON.")
            return

        start_class = request.get("start_class")
        goal_class = request.get("goal_class")
        avoid_classes = request.get("avoid_classes", [])

        if not start_class or not goal_class:
            return

        if isinstance(avoid_classes, str):
            avoid_classes = [avoid_classes]

        self.plan_from_latest_vision(start_class, goal_class, avoid_classes)

    def plan_from_latest_vision(self, start_class, goal_class, avoid_classes):
        data = self.latest_vision_data
        objects = data.get("objects", [])
        margin = float(self.get_parameter("planning_margin_m").value)

        start_obj = self.find_first_object(objects, start_class)
        goal_obj = self.find_first_object(objects, goal_class)

        if not start_obj or not goal_obj:
            self.get_logger().warn(f"Start ({start_class}) ili Goal ({goal_class}) nije pronađen!")
            return

        start = (float(start_obj["x"]), float(start_obj["y"]))
        goal = (float(goal_obj["x"]), float(goal_obj["y"]))

        # Izvlačenje i ekspandiranje svih prepreka
        obstacles = []
        for obj in objects:
            if obj.get("class") in avoid_classes and "bbox" in obj:
                obstacles.append(self.expand_bbox(obj["bbox"], margin))

        # --- POKRETANJE NAPREDNOG PLANERA ---
        path = self.plan_global_path(start, goal, obstacles)

        output = {
            "frame": data.get("frame", "robot_base"),
            "units": data.get("units", "m"),
            "start_class": start_class,
            "goal_class": goal_class,
            "avoid_classes": avoid_classes,
            "path": [{"x": round(p[0], 4), "y": round(p[1], 4)} for p in path]
        }

        out_msg = String()
        out_msg.data = json.dumps(output)
        self.path_pub.publish(out_msg)
        self.get_logger().info(f"Uspješno izračunata optimalna putanja s {len(path)} točaka.")

    def find_first_object(self, objects, class_name):
        for obj in objects:
            if obj.get("class") == class_name:
                return obj
        return None

    def expand_bbox(self, bbox, margin):
        x_min = min(float(bbox["x_min"]), float(bbox["x_max"]))
        x_max = max(float(bbox["x_min"]), float(bbox["x_max"]))
        y_min = min(float(bbox["y_min"]), float(bbox["y_max"]))
        y_max = max(float(bbox["y_min"]), float(bbox["y_max"]))
        return {
            "x_min": x_min - margin,
            "y_min": y_min - margin,
            "x_max": x_max + margin,
            "y_max": y_max + margin,
        }

    # --- NOVI GLOBALNI ALGORITAM UPRAVLJAN GRAFOM VIDLJIVOSTI I A* PROLAZOM ---
    def plan_global_path(self, start, goal, obstacles):
        # Ako nema prepreka ili linija start->goal ništa ne siječe, idi ravno (2 točke)
        if not any(self.segment_intersects_bbox(start, goal, obs) for obs in obstacles):
            return [start, goal]

        # 1. Sakupi sve sigurne čvorove grafa (start, goal + svi važeći kutovi oko kutija prepreka)
        nodes = [start, goal]
        for obs in obstacles:
            corners = [
                (obs["x_min"], obs["y_min"]),
                (obs["x_max"], obs["y_min"]),
                (obs["x_max"], obs["y_max"]),
                (obs["x_min"], obs["y_max"]),
            ]
            for corner in corners:
                # Dodajemo kut u graf samo ako se slučajno ne nalazi unutar Neke DRUGE prepreke
                if not any(self.point_inside_bbox(corner, other) for other in obstacles if other != obs):
                    nodes.append(corner)

        # Ukloni duplikate ako se prepreke preklapaju
        nodes = list(set(nodes))

        # 2. Izgradi susjedstvo (Adjacency list) - spoji sve čvorove koji se međusobno "vide"
        graph = {node: [] for node in nodes}
        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                n1 = nodes[i]
                n2 = nodes[j]
                
                # Provjera siječe li ravna crta između n1 i n2 bilo koju prepreku
                intersect = False
                for obs in obstacles:
                    if self.segment_intersects_bbox(n1, n2, obs):
                        intersect = True
                        break
                
                if not intersect:
                    dist = self.distance(n1, n2)
                    graph[n1].append((n2, dist))
                    graph[n2].append((n1, dist))

        # 3. Pokreni standardni A* algoritam nad generiranim grafom vidljivosti
        path = self.astar_search(graph, start, goal)
        if path:
            return path
        
        # Sigurnosni povratak (fallback) ako A* iz nekog ekstremnog razloga ne nađe rješenje
        self.get_logger().error("A* nije uspio naći slobodan put! Vraćam izravnu liniju.")
        return [start, goal]

    def astar_search(self, graph, start, goal):
        # Jednostavna i brza implementacija A* pretraživanja
        open_set = {start}
        came_from = {}

        g_score = {node: float('inf') for node in graph}
        g_score[start] = 0.0

        f_score = {node: float('inf') for node in graph}
        f_score[start] = self.distance(start, goal)

        while open_set:
            # Pronađi čvor u open_set s najmanjim f_score
            current = min(open_set, key=lambda node: f_score[node])

            if current == goal:
                # Rekonstruiraj putanju unatrag
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            open_set.remove(current)

            for neighbor, weight in graph[current]:
                tentative_g_score = g_score[current] + weight
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.distance(neighbor, goal)
                    if neighbor not in open_set:
                        open_set.add(neighbor)
        return None

    # --- POMOĆNE GEOMETRIJSKE FUNKCIJE (Zadržane i optimizirane) ---
    def segment_intersects_bbox(self, p1, p2, bbox):
        # Ako je neka točka unutar prepreke, segment je siječe
        if self.point_inside_bbox(p1, bbox) or self.point_inside_bbox(p2, bbox):
            return True

        # Rubovi kutije prepreke
        x_min, y_min, x_max, y_max = bbox["x_min"], bbox["y_min"], bbox["x_max"], bbox["y_max"]
        corners = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
        edges = [
            (corners[0], corners[1]),
            (corners[1], corners[2]),
            (corners[2], corners[3]),
            (corners[3], corners[0]),
        ]

        for e1, e2 in edges:
            if self.segments_intersect(p1, p2, e1, e2):
                return True
        return False

    def point_inside_bbox(self, p, bbox):
        # Malo smanjena tolerancija unutrašnjosti (za 0.1 mm) kako rubni kutovi ne bi ispali "unutar"
        return (bbox["x_min"] + 1e-4) <= p[0] <= (bbox["x_max"] - 1e-4) and \
               (bbox["y_min"] + 1e-4) <= p[1] <= (bbox["y_max"] - 1e-4)

    def segments_intersect(self, p1, p2, q1, q2):
        def orientation(a, b, c):
            value = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
            if abs(value) < 1e-4: return 0
            return 1 if value > 0 else 2

        def on_segment(a, b, c):
            return min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])

        o1, o2 = orientation(p1, p2, q1), orientation(p1, p2, q2)
        o3, o4 = orientation(q1, q2, p1), orientation(q1, q2, p2)

        if o1 != o2 and o3 != o4: return True
        if o1 == 0 and on_segment(p1, q1, p2): return True
        if o2 == 0 and on_segment(p1, q2, p2): return True
        if o3 == 0 and on_segment(q1, p1, q2): return True
        if o4 == 0 and on_segment(q1, p2, q2): return True
        return False

    def distance(self, p1, p2):
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()