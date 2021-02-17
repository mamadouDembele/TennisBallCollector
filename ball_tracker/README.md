# Tennis Ball Collector

## Présentation du paquet : ball_tracker 

L'objectif ici est grâce à la camera zénithale de repérer la position (x, y) de l'ensemble des balles sur le court et du robot. 

### Projection et Homographie

Pour avoir une position précise nous avons effectué une homographie de l'image issue de la caméra zénithale afin d'obtenir une image où toutes les pixels appartiennent au court de tennis avec une préservation des proportions. La bibliothèque OpenCV nous donne l'homographie. 

### Segmentation HSV

Nous utilisons une chaine de traitement pour obtenir le masque des balles, on isole les balles dans l'espace HSV, on effectue des opérations morphologiques pour avoir le bon nombre de balles. Nous effectuons le même process pour trouver la localisation du robot. 

Ensuite, grâce à la méthode des moments nous pouvons localiser le centre de la balle. Nous publierons le centre de chaque balles dans un topic de type PoseArray. 


## Démarrer la simulation

```bash
colcon build --symlink-install

. install/setup.bash

ros2 launch tennis_court tennis_court.launch.py

# Balls Localization 

ros2 run ball_tracker ball_tracker.py

```


## Groupe

### Membres

* **Jules Berhault** - 
* **Quentin Brateau** -  [Teusner](https://github.com/Teusner) :sunglasses:
* **Paul-Antoine Le Tolguennec** - [Paul-antoineLeTolguenec](https://github.com/Paul-antoineLeTolguenec)
* **Gwendal Priser** - [gwendalp](https://github.com/gwendalp) :ocean:

### Gestion de projet

Voici le lien [Taiga](https://tree.taiga.io/project/gwendalp-tennis-ball-collector/timeline).
https://tree.taiga.io/project/gwendalp-tennis-ball-collector/timeline


## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.

### Package `robot_tennis`

Le dossier `robot_tennis` est le dossier contenant la description du robot implémenté ppour résoudre le problème de récupération des balles.
Les fichiers de descriptions contiennent principalement:
* L'architecture du robot mobile 
* Les capteurs embarqués
* Le collecteur de balle

### Package `robot_tennis_controller`

Le dossier `robot_tennis_controller` les algorithmes de navigations du robot.
La partie décisionnelle du robot se divise 3 partie principales:
* Detections des balles
* Ordonnancement des balles à récupérer
* Récupération des balles 



### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.