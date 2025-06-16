# ROS LiDAR Examples

Примеры кода для работы со сканами LiDAR.

## Требования

1. Ubuntu 20.04 (ROS Noetic).
2. Python 3.8

## Как запустить?

### Эмуляция приближения и отдаления от препятствия

```shell
rosrun ros_lidar_examples fake_scan_publisher.py
```
В скрипте есть возможность изменить параметры - параметры LiDAR, скорость движения, размеры и расстояние до препятствия.

### Вычисление движения

```shell
rosrun ros_lidar_examples subscriber.py
```

Для визуализации запустите RViz:
```shell
rosrun rviz rviz -d rviz/default.rviz
```
