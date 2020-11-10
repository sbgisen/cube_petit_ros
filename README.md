# raspberrypi_cube_moc

Cube Petit is a desktop-sized Cuboid-kun that is intended to be sold at low cost and widely distributed around the world.

Cube Petit was born to coexist in people's living spaces, serve many people, and make many people smile.

Cube Petit is self-driving and can be charged by itself.

There are ivory, clear blue, and yellow color variations.

(日本語訳：キューブプチ可愛いでしょ！！！)

## Packages

1. cube_petit_ros
1. cube_petit_hardware_interface
1. cube_petit_bringup
1. cube_petit_description
1. cube_petit_gazebo
1. cube_petit_navigation


## Invtoduction1(for cube_petit/Ubuntu18)

```
git clone cube_petit_setup
cd cube_petit_setup
setupscript
```

## Introduction2(for other PC/Ubuntu16or18)
```
git clone @@@@@@@
cd cube_petit_ros/cube_petit_ros
catkin bt
source ~/.bashrc
```
## Usage

terminal1
```
roslaunch cube_petit_bringup cube_petit_bringup.launch
```

terminal2
```
roslaunch cube_petit_navigation cube_petit_naviagation.launch
```

## Hardware Changes

## cube_petit v2

* 2020/1~2020/12

* HARDWARE
    * Intel mini NUC PC
    * 5 inch display
    * Life-po4 battery
    * dji Motors

## cube_petit v1

* 2019/10~2019/12
* default small box-sized type
* almite frame and acrilic plate
* [how to set up](./README.md)
* 220[mm]*220[mm]*270[mm]
* crrspnd gazebo/rviz
* target selling price 50,000(en)

* SOFTWARE
    * Ubuntu18.04LTS
    * ROS melodic

* HARDWARE
    * raspberryPi4
    * Real SENSE SR502
    * 2D LiDER
    * Dynamixel MX-28
    * 5.5inch display
    * battery

* function(ROS package, etc.)
    * teleop(joy)
    * lider
    * motor
    * display
    * bring_up


## Author

* Airi Yokochi
* Softbank corp.
* airi.yokochi@g.softbank.co.jp

## Licence

* MIT
