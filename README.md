# Fennec Jet Racer
Bitte alle Unterlagen bzw Code entsprechend der vorgegebenen Struktur ablegen.
Grundsätzlich im Master Branch einchecken


# Installation

#### Installation der Software

Da wir ROS Melodic auf Ubuntu 18.04 benutzen, verweisen wir zur Installation von ROS auf das Wiki, welches eine sehr ausführliche Einleitung zur Installation und Anwendung beinhaltet: http://wiki.ros.org/ROS/Installation Wir empfehlen die version "ros-melodic-full-desktop"

#### Installation der benötigten Pakete

Nach dem ROS installiert wurde, müssen einige Pakete, falls nicht vorhanden, nachinstalliert werden:

##### ROS Navigation (AMCL zur Lokalisierung eingebunden):

`sudo apt install ros-melodic-navigation`

`sudo apt install ros-melodic-nav2d`

Mit diesen Paketen wird die Steuerung des Roboters erst möglich. 

##### Gmapping zur Erstellung der Map/Karte in RViz

`sudo apt install ros-melodic-gmapping`

##### Anbinden des LiDARs:

`sudo apt install ros-melodic-rplidar-ros`

##### Anbinden der Intel Realsense und Konvertierung von Tiefendaten in laserscan:

`sudo apt install ros-melodic-librealsense2`

`sudo apt install ros-melodic-depthimage-to-laserscan`

##### OpenCV in Verbindung mit ROS:

`sudo apt install ros-melodic-cv-bridge`



# Benutzung

#### Verbinden des Jetson Racers mit ROS

Um den Jetson Racer zu starten, muss der kleine Schalter am Waveshare- Board betätigt werden. Damit der Jetson Racer über ROS gesteuert werden kann, müssen erstmal einige Nodes (Skripte im scripts-Ordner) gestartet werden.

Um die ROS Controller und die Visualisierung (RViz) zu starten muss folgende Zeile im Terminal ausgeführt werden:

`roslaunch jetson_racer gazebo.launch`

Anschließend wird der Controller mit der Hardware über folgenden Command verbunden:

`rosrun jetson_racer racecar.py`

Jetzt ist es schon möglich, den Jetson Racer über /cmd_vel messages zu steuern. Um den Jetson Racer mit einem GamePad zu steuern, muss in einem neuen Terminal folgende Node gestartet werden. Davor sollte das GamePad jedoch an den Jetson Nano angeschlossen werden. Dies geschieht über den "on"- Button auf der Unterseite des GamePads und über Bluetooth:

`rosrun jetson_racer teleop_gamepad.py`

Tastatursteuerung ist auch möglich. Dafür muss, anstelle des obigen Commands, folgender ausgeführt werden:

`rosrun jetson_racer teleop.py`

#### Starten der Intel Realsense

Um die Farb- und Tiefenkamera (die Intel Realsense) zu starten muss in einem neuen Terminal folgendes eingegeben werden:

`roslaunch realsense2_camera rs_camera.launch`

#### Starten des LiDARs

Um den Laserscanner zu starten und mit ROS zu verbinden muss folgender Befehl im Terminal ausgeführt werden:

`roslaunch rplidar_ros view_rplidar.launch`

