# Fennec Jet Racer
Bitte alle Unterlagen bzw Code entsprechend der vorgegebenen Struktur ablegen.
Grundsätzlich im Master Branch einchecken


# Installation


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

