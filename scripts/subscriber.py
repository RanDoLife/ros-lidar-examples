#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
from typing import Optional, Tuple

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class ObstacleMovementDetector:
    def __init__(self):
        rospy.init_node('obstacle_movement_detector')

        # Параметры
        self.prev_distance: Optional[float] = None
        self.threshold = 0.02  # порог стабильности (в метрах)
        self.fov_degrees = 15  # сектор обзора ±15°

        # Паблишеры
        self.marker_pub = rospy.Publisher('/movement_marker', Marker, queue_size=1)
        self.movement_pub = rospy.Publisher('/obstacle_movement', String, queue_size=1)

        # Подписка
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, msg: LaserScan) -> None:
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        frontal_ranges = self.get_frontal_sector(ranges, angles)
        if frontal_ranges.size == 0:
            rospy.logwarn("Нет валидных данных в фронтальном секторе")
            return

        min_distance = np.min(frontal_ranges)
        movement, delta = self.detect_movement(min_distance)

        self.log_movement(movement, min_distance, delta)
        self.publish_marker(movement, min_distance)
        self.movement_pub.publish(movement)

        # Доп: скорость изменения расстояния
        dt = msg.scan_time
        if dt > 0:
            speed = delta / dt
            rospy.loginfo(f"Speed: {speed:.3f} m/s")

    def get_frontal_sector(self, ranges: np.ndarray, angles: np.ndarray) -> np.ndarray:
        fov_rad = np.radians(self.fov_degrees)
        mask = (np.abs(angles) <= fov_rad)
        sector = ranges[mask]

        # Фильтрация плохих значений
        valid = ~np.isnan(sector) & ~np.isinf(sector)
        filtered = sector[valid]
        return filtered

    def detect_movement(self, current_distance: float) -> Tuple[str, float]:
        movement = "stable"
        delta = 0.0

        if self.prev_distance is not None:
            delta = self.prev_distance - current_distance
            if abs(delta) < self.threshold:
                movement = "stable"
            elif delta > 0:
                movement = "approaching"
            else:
                movement = "receding"

        self.prev_distance = current_distance
        return movement, delta

    def log_movement(self, movement: str, distance: float, delta: float) -> None:
        colors = {
            "approaching": "\033[91m",  # Красный
            "receding": "\033[92m",     # Зелёный
            "stable": "\033[94m"        # Синий
        }
        reset = "\033[0m"

        rospy.loginfo(
            f"{colors[movement]}{movement.upper()}{reset} | "
            f"Distance: {distance:.2f}m | "
            f"Change: {delta:.3f}m"
        )

    def publish_marker(self, movement: str, distance: float):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "movement_arrow"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        start_point = Point(0, 0, 1.2)
        end_point = Point(-0.3 if movement == "approaching" else 0.3, 0, 1.2)

        marker.points.append(start_point)
        marker.points.append(end_point)

        marker.scale.x = 0.05
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        if movement == "approaching":
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Красный
        elif movement == "receding":
            marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Зелёный
        else:
            marker.color = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # Серый

        self.marker_pub.publish(marker)

def main() -> None:
    ObstacleMovementDetector()
    rospy.spin()

if __name__ == '__main__':
    main()