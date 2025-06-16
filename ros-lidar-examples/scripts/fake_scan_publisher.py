#!/usr/bin/env python3
# encoding: utf-8

import rospy
import random
import numpy as np

from sensor_msgs.msg import LaserScan

class WallScanPublisher:
  def __init__(self):
    rospy.init_node('fake_scan_publisher')
    self.pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    # Параметры сканирования (аналогично URG-04LX)
    self.num_readings = 720          # Количество лучей
    self.angle_min = -np.radians(90) # -90 градусов
    self.angle_max = np.radians(90)  # +90 градусов
    self.angle_increment = np.pi/720 # Шаг угла
    self.range_min = 0.05            # Минимальная дистанция (м)
    self.range_max = 4.0             # Максимальная дистанция (м)

    # Параметры стены
    self.wall_distance = 1.0      # Начальная дистанция до стены (м)
    self.movement_speed = 0.025     # Скорость движения стены (м/итерацию)
    self.direction = 1            # 1 = стена приближается, -1 = отдаляется

    self.rate = rospy.Rate(10)    # 10 Hz

  def generate_scan(self) -> LaserScan:
    scan = LaserScan()
    scan.header.stamp = rospy.Time.now()
    scan.header.frame_id = 'laser'
    scan.angle_min = self.angle_min
    scan.angle_max = self.angle_max
    scan.angle_increment = self.angle_increment
    scan.range_min = self.range_min
    scan.range_max = self.range_max

    # Обновляем позицию стены
    self.update_wall_position()

    # Генерируем данные скана
    scan.ranges = [
      self.get_wall_distance(angle)
      for angle in [
        self.angle_min + i * self.angle_increment
        for i in range(self.num_readings)
      ]
    ]

    return scan

  def update_wall_position(self):
    """Изменяет дистанцию до стены"""
    self.wall_distance += self.movement_speed * self.direction

    # Меняем направление при достижении границ
    if self.wall_distance < 0.5:
      self.direction = 1
      rospy.loginfo("Стена начала отдаляться")
    elif self.wall_distance > 3.0:
      self.direction = -1
      rospy.loginfo("Стена начала приближаться")

  def get_wall_distance(self, angle: float) -> float:
    """Возвращает расстояние до стены для заданного угла"""
    # Стена обнаруживается только в узком секторе прямо перед лидаром (±5°)
    if np.abs(angle) < np.radians(10):
      # Добавляем небольшой шум для реалистичности
      return self.wall_distance + random.uniform(-0.005, 0.005)

    # В остальных направлениях - нет препятствий
    return self.range_max

  def run(self):
    while not rospy.is_shutdown():
      self.pub.publish(self.generate_scan())
      self.rate.sleep()

def main() -> None:
  publisher = WallScanPublisher()
  publisher.run()

if __name__ == '__main__':
  main()