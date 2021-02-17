# Robot_tennis_gazebo

Package ROS2 pour la création du robot 2 roues dans rviz.

# Description du package

- **launch:** Dossier contenant trois launch files *gazebo.launch.py*, *gazebo_foxy.launch.py* et *spawner.launch.py*. les deux premiers lance le robot dans gazebo. qui permet de lancer rviz et d'afficher le robot.

# Utilisation du Package

- Lancer le fichier sur eloquent:
```shell
ros2 launch robot_tennis_gazebo gazebo.launch.py
```

- Lancer le fichier sur eloquent:
```shell
ros2 launch robot_tennis_gazebo gazebo_foxy.launch.py
```

# Autre utilisation

Lorsqu'on veut afficher le robot sur la piste de tennis

- Lancer dans un premier terminal,
```shell
ros2 launch tennis_court tennis_court.launch.py
```

- Dans un second terminal

```shell
ros2 launch robot_tennis_gazebo spawner.launch.py
```
