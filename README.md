# Fennec Jet Racer
#### Teammitglieder
- Umut Uzunoglu
- Hien Anh Ngyuen Manh 
- Son Khue Ngyuen 
- Lukas Evers

## Videopräsentation Sprint 1: [Youtube](https://youtu.be/tqEe7d1nCfg)

## Powerpoint-Präsentation Sprint 2: [Google Docs](https://drive.google.com/file/d/15EYwAGZ4gDesyfAvPCeXksNx87YFiDXL/view?usp=sharing)

## Präsentation Sprint 3 mit Embedded Videos: [Google Docs](/cmd_vel [geometry_msgs/Twist])


# Installation

#### Installation der Software

Da wir ROS Melodic auf Ubuntu 18.04 benutzen, verweisen wir zur Installation von ROS auf das Wiki, welches eine sehr ausführliche Einleitung zur Installation und Anwendung beinhaltet: http://wiki.ros.org/ROS/Installation Wir empfehlen die version "ros-melodic-full-desktop"

Ubuntu 18.04 ist das Betriebssystem des Nvidia Jetpack Images welches wir verwenden. Das Image ist speziell für die Verwendung mit einem Jetson Nano entwickelt worden.

#### Setup des Jetracers

##### Erstellung des SD Karten Images
- Mindestgröße der SD-Karte muss 64GB sein 
- Mindestgeschwindigkeit der SD-Karte sollte mind. Klasse 10 sein

1. Download JetRacer Image von Nvidia: [Image](https://drive.google.com/open?id=1wXD1CwtxiH5Mz4uSmIZ76fd78zDQltW_)
2. SD-Karte mittels Balena Etcher Software mit dem Nvidia Image beschreiben

##### Starten des Jetson Nano
- Der SD Karten-Slot befindet sich auf der Unterseite des Jetson Nano
- Die initale Einrichtung kann über die Verbindung mit USB oder per angeschlossener Peripherie geschehen (Bildschirm, Tastatur, Maus)

- USB-Verbindung folgendermaßen:
  1. `192.168.55.1:8888` im Browser aufrufen
  2. Passwort: "jetson"
 
- Neuen Terminal öffnen
- Mit einem Wifi Netzwerk verbinden mit dem Befehl:


    `sudo nmcli device wifi list`

    `sudo nmcli device wifi connect <ssid_name> password <password>`

#### Installation Nvidia & Waveshare Pakete


##### Update Jetcard Python
```
cd 
sudo mkdir ws
cd ws
sudo git clone https://github.com/waveshare/jetcard
sudo cp jetcard/jetcard/ads1115.py ~/jetcard/jetcard/
sudo cp jetcard/jetcard/ina219.py ~/jetcard/jetcard/
sudo cp jetcard/jetcard/display_server.py ~/jetcard/jetcard/
sudo cp jetcard/jetcard/stats.py ~/jetcard/jetcard/
cd ~/jetcard
sudo pip3 uninstall jetcard -y
sudo reboot  #reboot and then install
cd ~/jetcard
sudo python3 setup.py install
```

##### Installation JetCam Python

```
cd 
sudo git clone https://github.com/NVIDIA-AI-IOT/jetcam
cd jetcam
sudo python3 setup.py install
```

##### Installation torch2trt Python Package

```
cd 
git clone https://github.com/NVIDIA-AI-IOT/torch2trt
cd torch2trt
sudo python3 setup.py install
```

##### Installation JetRacer Package

```
cd
git clone https://github.com/waveshare/jetracer
cd jetracer
sudo python3 setup.py install
```

##### Power Mode wechseln 

`sudo nvpmodel -m1`

#### Installation der benötigten ROS-Pakete

Nach dem ROS installiert wurde, müssen einige Pakete, falls nicht vorhanden, nachinstalliert werden:
Die Liste der installierten Pakete kann mit dem Befehl 
`rospack list`
überprüft werden. (Wichtig: Der aktuelle Workspace und die ROS-Installation müssen gesourced werden, siehe ROS Installation)

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

#### Erstellen eines Catkin Workspaces

```
cd
mkdir fennec_ws
cd ~/fennec_ws
mkdir src
cd src
git clone THIS REPO TO DO: link einfügen
To Do: git clone navigtion, rplidar, gmapping, etc..
cd ~/fennec_ws
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make (Dauer 5-10 Minuten)
```

# Benutzung

#### Verbinden des Jetson Racers mit ROS

Um den Jetson Racer zu starten, muss der kleine Schalter am Waveshare- Board betätigt werden. Damit der Jetson Racer über ROS gesteuert werden kann, müssen erstmal einige Nodes (Skripte im scripts-Ordner) gestartet werden.

Um eine Verbindung zum Jetson-Board herzustellen muss sich der Jetson in ein WLAN Netzwerk einwählen. Dies sollte zu Beginn einmalig erledigt werden, indem man Bildschirm, Maus und Tastatur anschließt.
Nachdem der Jetson die Verbindung hergestellt hat kann die zugewiesene IPv4 Adresse vom Status LED Display auf der Rückseite des Jetson abgelesen werden. 
Die Verbindung zum Jetson erfolgt über SSH (Secure Shell) mit dem Befehl:

`ssh jetson@[IP-Adresse]`

`Passwort: "jetson"`

Nun kann man über den Terminal die benötigten ROS Launch Files auf dem Jetson starten. Anzumerken ist, dass im Augenblick mehrere Terminals erstellt werden müssen. D.h. man verbindet sich jedes mal erneut in einem neuen Terminal über SSH mit dem Jetson.

Um ROS Funktionalitäten nutzen zu können, muss die zentrale ROS Instanz "roscore" gestartet werden, welche primär die Registrierung der Publisher/Subscriber auf verschiedenen Topics und der Service/Action Server ausführt.

_Terminal 1:_ `roscore`

Um die ROS Controller und die Visualisierung (RViz) zu starten muss folgende Zeile ausgeführt werden:

_Terminal 2:_ `roslaunch jetson_racer gazebo.launch`

Anschließend wird der Controller mit der Hardware über folgenden Command verbunden:

_Terminal 3:_ `rosrun jetson_racer racecar.py`

Jetzt ist es schon möglich, den Jetson Racer über /cmd_vel messages zu steuern. Um den Jetson Racer mit einem GamePad zu steuern, muss in einem neuen Terminal folgende Node gestartet werden. Davor sollte das GamePad jedoch an den Jetson Nano angeschlossen werden. Dies geschieht über den "on"- Button auf der Unterseite des GamePads und über Bluetooth:

_Terminal 4:_ `rosrun jetson_racer teleop_gamepad.py`

Tastatursteuerung ist auch möglich. Dafür muss, anstelle des obigen Commands, folgender ausgeführt werden:

_Alternative Terminal 4:_`rosrun jetson_racer teleop.py`

#### Starten der Intel Realsense

Um die Farb- und Tiefenkamera (die Intel Realsense) zu starten muss in einem neuen Terminal  folgendes eingegeben werden:

_Terminal 5:_ `roslaunch realsense2_camera rs_camera.launch`

#### Starten des LiDARs

Um den Laserscanner zu starten und mit ROS zu verbinden muss folgender Befehl im Terminal ausgeführt werden:

_Terminal 6:_ `roslaunch rplidar_ros view_rplidar.launch`

#### Starten des Navigation Stacks

Da die Steuerung des Fennecs über ROS Navigation erfolgen kann, sind die Schritte von Terminal 4 bis Terminal 6 nicht mehr nötig. Dafür müssen wir aber ROS Navigation ausführen:

Nachdem RViz gestartet wurde, wird der Laserscanner über folgenden Befehl gestartet. Dabei wird auch AMCL mitgestartet.

_Terminal 4:_ `roslaunch realsense2_camera amcl_lidar.launch`

Nun starten wir SLAM- Gmapping, diese Node wird bis zum Starten der LiDAR nichts machen, da sie noch auf Scandaten wartet.

_Terminal 5:_ `rosrun gmapping slam_gmapping scan:=scan`

Schließlich kann auch der Kern des Navigation Stacks gestartet werden:

_Terminal 6:_ `roslaunch jetson_racer move_base.launch`


In Rviz, oben auf der Toolbar, kann man auf "2d Nav Goal" klicken und eine Pose dem Navigation Stack übergeben, der daraufhin einen Pfad hin plant und auch ausführt. Einige Parameter müssen noch genauer betrachtet und "fine tuned" werden.

#### Starten der Ballverfolgung

Wichtig bei der Ballverfolgung ist, dass der Ball erkannt wird, dass die Koordinaten des erkannten Balles im /cmd_vel Topic als Twist Messages umgewandelt wird und dass diese Messages auch an der Hardware ankommen. Zur Ballerkennung muss noch die Kamera gestartet werden. Mit der ROS Instanz zusammen brauchen wir insgesamt 5 Terminals, wobei im späteren Verlauf des Projektes einige zusammengefasst werden. So ist es leichter, Fehler beim Starten zu identifizieren. 

Das erste Terminal soll nur unsere ROS Instanz aufrufen. Nicht genauer spezifizierte Terminals können auch über das Multiple Machines Setup vom Laptop/PC aus aufgerufen werden. Dadurch spart man Ressourcen auf dem Racer, jedoch könnte die dadurch entstandene Latenz die Übertragung der Messages negativ beeinflussen. 

_Terminal 1:_ `roscore`

Auf dem Fennec wird die Übergabe von Twist Messages an Hardware gestartet:

_Terminal 2:_ `rosrun jetson_racer racecar.py`

Außerdem muss auf dem Fennec auch die Kamera gestartet werden, da sie direkt am Fennec angeschlossen wird:

_Terminal 3:_ `roslaunch realsense2_camera rs_camera.launch`

Nun starten wir die Ballerkennung:

_Terminal 4:_ `rosrun opencv find_ball.py`

Und schließlich wird die Verfolgung gestartet:

_Terminal 5:_ `rosrun opencv chase.py

Wenn jetzt ein blauer Ball vor der Kamera gehalten wird, wird sich der Fennec zum Ball bewegen und entsprechend auch korrigieren.
