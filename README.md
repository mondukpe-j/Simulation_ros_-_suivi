# ros_sv — ROS 2 / MAVROS / Gazebo

Intégration de la pile PX4 dans ROS 2 via MAVROS, simulation Gazebo Harmonic avec caméra embarquée, nœuds de navigation et de perception.

## Construire et lancer le conteneur

```bash
cd ROS_drone/code/ros_sv
docker compose build
docker compose run ros2-mavros
```

Le dossier `ros_sv/` est monté dans le conteneur — c'est le workspace ROS 2 (packages `navigation` et `drone_perception`) et il contient aussi la copie de PX4-Autopilot.

## Récupérer PX4-Autopilot (une seule fois)

```bash
cd ROS_drone/code/ros_sv
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
```

## Lancer le simulateur (SITL + Gazebo)

Terminal 1, dans le conteneur :

```bash
cd /workspace/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500_mono_cam
```

Le modèle `x500_mono_cam` inclut la caméra nécessaire à la partie perception.

## Lancer MAVROS

Terminal 2 (`docker exec -it ros_sv bash`)  (le conteneur s'appelle également ros_sv) :

```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
```

Vérifier la connexion : `ros2 topic echo /mavros/state` doit afficher `connected: true`.

## Lancer les ponts caméra Gazebo → ROS 2

Terminal 3 et 4 (un par pont, à garder actifs) :

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image

ros2 run ros_gz_bridge parameter_bridge \
    /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

## Lancer les ponts caméra Gazebo → ROS 2

Terminal 3 et 4 (un par pont, à garder actifs) :

```bash
ros2 run ros_gz_bridge parameter_bridge \
    /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image

ros2 run ros_gz_bridge parameter_bridge \
    /world/default/model/x500_mono_cam_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo
```

## Compiler les packages ROS 2

```bash
cd /workspace/src/ros_sv
colcon build
source install/setup.bash
```

## Lancer les nœuds

Décollage :

```bash
ros2 run navigation arm_takeoff_node
```

Une fois le drone en l'air, dans un nouveau terminal, mission + RTL :

```bash
ros2 run navigation mission_node
```

Détection ArUco (nécessite les ponts caméra actifs) :

```bash
ros2 run drone_perception aruco_detector_node
```

## Remarques

- Toujours vérifier `ros2 topic info -v <topic>` avant d'écrire une nouvelle souscription — les topics MAVROS n'ont pas tous le même profil QoS, une incompatibilité fait échouer la réception silencieusement.