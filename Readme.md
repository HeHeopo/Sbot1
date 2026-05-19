# Snowbot
ROS 2 пакет для робота с mecanum колесами с симуляцией Gazebo Harmonic.
## Из чего состоит
- Немного убогий корпус
- Mecanum колеса (4 колеса с omnidirectional движением)
- LDS лидар
- Камера
- IMU (Inertial Measurement Unit)
- EKF (Extended Kalman Filter) для sensor fusion
## Зависимости
- ROS 2 Jazzy
- Gazebo Harmonic (8)
- ros_gz_sim
- ros_gz_bridge
- ros_gz_image
- robot_localization
## Сборка
```bash
cd ~/ros2_ws
colcon build --packages-select snowbot123
source install/setup.bash
```
## Запуск
```bash
ros2 launch snowbot123 robot.launch.py
```
### Параметры запуска
- `world` - путь к world файлу (по умолчанию: worlds/wow.sdf)
- `rviz` - запустить RViz (по умолчанию: True)
- `use_sim_time` - использовать симуляционное время (по умолчанию: true)
## Топики ROS
| Топик | Тип | Описание |
|-------|-----|---------|
| `/cmd_vel` | geometry_msgs/Twist | Управление роботом |
| `/odom` | nav_msgs/Odometry | Одометрия от Gazebo |
| `/odometry/filtered` | nav_msgs/Odometry | Одометрия от EKF |
| `/joint_states` | sensor_msgs/JointState | Состояние колес |
| `/scan` | sensor_msgs/LaserScan | Скан лидара |
| `/imu` | sensor_msgs/Imu | Данные IMU |
| `/camera_info` | sensor_msgs/CameraInfo | Информация камеры |
| `/camera_image` | sensor_msgs/Image | Изображение |
## Из чего состоит пакет
**URDF** (Universal Robot Description Format) или **SDF** (Simulation Description Format). Эти форма-
ты позволяют задавать геометрию, массу, инерцию и другие параметры как для
отдельных компонентов, так и для всей системы целиком.
# URDF
- **link** описывает кинематические и динамические свойства жёсткой инерционной детали или узла робота (абсолютно твёрдого тела).
- **visual** описывает визуальные свойства <link>. То есть, как выглядит деталь робота. Узел робота может быть условно описан геометрически как куб, цилиндр или сфера, или же задан полигональной 3D-моделью.
- **collision** описывает упрощённую геометрию <link>. Данный элемент используется для задания хитбоксов и областей, которые нужны для физических расчетов во время симуляции.
- **inertial** описывает инерционные свойства <link>. Элемент задаёт массу детали или узла робота, центр масс и тензор инерции в виде матрицы 3×3.
- **joint** описывает шарнир, сочленение или соединение двух жёстких частей (двух элементов <link>). В этом элементе определяется тип соедининия двух деталей, их кинематика и динамика, пределы безопасного движения, физическое демпфирование и параметры трения.
- **transmission** описывает взаимосвязь между <joint> и конкретным приводом. Например, один мотор робота может управлять несколькими сочленениями.
# SDF
- **sdf** (Root): Корневой элемент, указывающий версию формата.
- **world** (Мир): Глобальные параметры симуляции, включая гравитацию, освещение (тени, небо) и физический движок (например, ODE, Bullet).
- **model** (Модель): Описание роботов или объектов. Включает:
  **link** (Звено): Инерционные характеристики (масса, инерция), визуальные модели (внешний вид) и коллизии (физические границы).
  **joint** (Сочленение): Типы связей между звеньями (шарнирные, призматические) и ограничения движения.
  **sensor** (Датчики): Камеры, лидары, IMU и другие сенсоры, привязанные к модели.
- **plugin** (Плагины): Сторонние программные модули для управления поведением роботов или среды.
# Bridge
Для передачи данных между gazebo и ROS 2 и интеграции датчиков использу-
ется мост ros_gz_bridge, который выступает посредником, подписываясь на топи-
ки в одной системе и публикуя их в другой.

# GZ -> ROS (от Gazebo к ROS)
- ros_topic_name: "/odom"           # Имя топика в ROS
  gz_topic_name: "/odom"           # Имя топика в Gazebo
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
# ROS -> GZ (от ROS к Gazebo)
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
Направления
- GZ_TO_ROS - из Gazebo в ROS
- ROS_TO_GZ - из ROS в Gazebo
- BIDIRECTIONAL - в обе стороны
Примеры топиков
Gazebo
/clock
odom
scan
cmd_vel
Запуск моста
ros_gz_bridge \
  --ros-args \
  -p config_file:=config/gz_bridge.yaml
Проверка
# Список мостов
ros2 bridge --help
gz topic -l
## TF tree
```
odom
 └── base_footprint
     └── base_link
          ├── front_left_wheel
          ├── front_right_wheel
          ├── rear_left_wheel
          ├── rear_right_wheel
          ├── lidar_link
          │    └── camera_link
          └── imu_link
```
## Управление роботом
```bash
# Публикация скорости (движение вперед)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
# Поворот на месте
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" -r 10
```
## Диагностика
```bash
# Проверка tf
rqt
# Проверка топиков
ros2 topic list
# Проверка odometry
ros2 topic echo /odom --once
# Проверка EKF
ros2 topic echo /odometry/filtered --once
```
## Структура пакета
```
snowbot123/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   ├── robot.launch.py      # Основной launch файл
│   ├── rsp.launch.py   # Robot state publisher
│   └── mapping.launch.py
├── urdf/
│   ├── bot_mecanum.urdf
│   ├── bot_mecanum.gazebo
│   └── materials.xacro
├── config/
│   ├── gz_bridge.yaml
│   └── ekf.yaml
├── worlds/
│   └── wow.sdf
├── rviz/
│   └── bot.rviz
└── meshes/
    └── *.STL
```
## Конфигурация EKF
EKF использует данные odometry и IMU для построения более точной оценки положения.
### Используемые данные:
- **odom** - одометрия от Gazebo (используется x, y, yaw)
- **imu** - данные IMU (используется angular velocity и orientation)
## Известные проблемы
1. **RTPS_TRANSPORT_SHM Error** - предупреждение о невозможно��ти использовать разделяемую память, не влияет на работу
2. **TF_OLD_DATA** - предупреждение о старых данных в tf, может возникать при запуске
## Лицензия
TODO
