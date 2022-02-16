# EMR22 - ROS und MoveIt!
Repsitorium zum Modul "Embedded Robotics" - EMR22 ab dem SS22.
Stichworte:  ROS1 - Noetic, Python, openCV, Gazebo, MoveIt!, UR3, UR5e

![rviz_plugin](https://jaspereb.github.io/UR5_With_ROS_Moveit_Tutorial/media/frontImg.png)


## Usage - Start der Umgebung mit UR5 mit Gripper ##
>$1 roslaunch ur5_moveit_config demo_gazebo.launch 



## Installation ##

Wird in den **~/ws_moveit/src** geclont und mit catkin build kompiliert


Nachdem Ubuntu Focal Fossa 20.4 (LTS) installiert wurde 
als erstes einen Workspace Ordner erstellen

1. Terminal öffnen mit STRG+ALT+T. Hinter dem Prompt $ die Befehle eingeben

>$ cd ~

>$ mkdir ws_moveit

>$ cd ws_moveit

>$ mkdir src

2. Klonen des Repositoriums nach catkin_ws/src
>$ cd ~/ws_moveit/src/

>$ git clone https://github.com/ProfJust/emr22.git

_ggf. ist es vorher erforderlich noch git zu installieren:_
>$ sudo apt install git

Jetzt sollte der Ordner emr22 geclont worden sein.


3. Erstellen des Moveit-Workspace mit Shellskript
>$ cd ~/ws_moveit/src/emr22/install_script

Nun ggf. erstmal ROS-Noetic installieren, dazu dem Skript 
die Ausführungsrechte geben und dann ausführen
>$ chmod +x ros_noetic_install_skript.sh

>$ ./ros_noetic_install_skript.sh 

4. Danach sämtliche für unseren Roboter benötigte Software-
Pakete installieren

>$ chmod +x  Install_MoveIt_on_ROS1.sh

>$ ./Install_MoveIt_on_ROS1.sh

5. ggf. noch die .bashrc - Datei konfigurieren
>$ gedit ~/.bashrc

Ergänze die Zeile
> source ~/ws_moveit/devel/setup.bash

6. Kompilieren mit dem catkin-Build-System
> cd ~/ws_moveit

> catkin build

> source ~/ws_moveit/devel/setup.bash

Falls catkin build nicht bekannt ist, 
versuchen Sie den Befehl in einem neuen Terminal zu starten.

Falls catkin build während der Kompilation abstürzt, prüfen Sie bitte, ob Ihr Speicher bzw. Sawp-Speicher ausreicht, da die Kompilation von MoveIt hier sehr anspruchsvoll ist. Ggf. den Swap Buffer vergrößern.

7. Nun sollte man die Panda Arm - Demo in RViZ starten können

>$ roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

In RViz ggf. ADD Diplay <moveit_ros_visualization>, damit man den Panda Arm auch sieht.

### UR5 und Gripper ###

8. Zum Schluss noch die URDFs für den UR5, den Gripper und die MoveIt-Konfiguration installieren.

>$ cd ~/ws_moveit/src/emr22/install_script

>$ chmod +x Install_UR_URDF_Gazebo.sh

>$ ./Install_UR_URDF_Gazebo.sh

