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
>$ chmod +x step1_ros_noetic_install_skript.sh

>$ ./step1_ros_noetic_install_skript.sh

4. Danach sämtliche für unseren Roboter benötigte Software-
Pakete installieren

>$ chmod +x  step2_Install_MoveIt_on_ROS1.sh

>$ ./step2_Install_MoveIt_on_ROS1.sh

5. Das Shell-Script sollte die die .bashrc - Datei konfiguriert haben, ansonsten
>$ gedit ~/.bashrc
Ergänze die Zeile
> source ~/ws_moveit/devel/setup.bash

Da catkin build bei der frischen Installation (noch) nicht bekannt ist, 
versuchen Sie den Befehl in einem neuen Terminal zu starten.
(Shell neu starten oder source ~/.bashrc)
> source ~/ws_moveit/devel/setup.bash

6. Jetzt sollte das Kompilieren mit dem catkin-Build-System funktionieren
> cd ~/ws_moveit
> catkin build

  Falls catkin build während der Kompilation abstürzt, prüfen Sie bitte, ob Ihr Speicher bzw. Sawp-Speicher ausreicht,
  da die Kompilation von MoveIt hier sehr anspruchsvoll ist. Ggf. den Swap Buffer vergrößern.

7. Installation der UR5 Simulation in Gazebo
  >$ cd ~/ws_moveit/src/emr22/install_script
  >$ chmod +x  step3_Install_UR_URDF_Gazebo.sh
  >$ ./step3_Install_UR_URDF_Gazebo.sh

8. Installation der MoveIt Gripper Configuration für Gazebo
  >$ cd ~/ws_moveit/src/emr22/install_script
  >$ chmod +x  step4_Install_UR5_Gripper_Moveit_Config_4_Gazebo.sh
  >$ ./step4_Install_UR5_Gripper_Moveit_Config_4_Gazebo.sh

9. Nun sollte man die UR5 Gazebo Simulation + RViZ starten können

>$ roslaunch 

In RViz ggf. ADD Diplay <moveit_ros_visualization>, damit man den Panda Arm auch sieht.

### UR5 und Gripper ###

8. Zum Schluss noch die URDFs für den UR5, den Gripper und die MoveIt-Konfiguration installieren.

>$ cd ~/ws_moveit/src/emr22/install_script

>$ chmod +x Install_UR5_Gripper_MoveIt _Config_4_Gazebo.sh

>$ ./Install_UR5_Gripper_MoveIt _Config_4_Gazebo.sh

TEST
