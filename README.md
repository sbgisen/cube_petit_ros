# cube_petit_ros

Cube Petit is a desktop-sized Cuboid-kun that is intended to be sold at low cost and widely distributed around the world.

Cube Petit was born to coexist in people's living spaces, serve many people, and make many people smile.

Cube Petit is self-driving and can be charged by itself.

There are ivory, clear blue, and yellow color variations.

(日本語訳：キューブプチ可愛いでしょ！！！)

## Packages

1. cube_petit_ros
2. cube_petit_ar_docking (auto charging)
3. cube_petit_bringup
4. cube_petit_description (urdf/xacro)
5. cube_petit_gazebo
6. cube_petit_hardware_interface (motor)
7. cube_petit_navigation
8. jbd_battery_monitor (battery monitor)
9.  navigation_goals
10. cube_petit_text_to_speech
11. cube_petit_speech_to_text

## Software
- Ubuntu18.04
- ROS(Melodic)

## Sensors(v3)
- LiDAR (LDS-50C)
- Depth Camera (Intel RealSense Depth Camera SR305)
- Infrared Camera(ELP-USBFHD01M-KRL156)
- Air pressure meter and 9-axis IMU(Witmotion wt901b)

## Invtoduction1(for cube_petit/Ubuntu18)

```
git clone git@github.com:AiriYokochi/setup_cube_petit.git
cd cube_petit_setup
source setup.bash
```

## Introduction2(for other PC/Ubuntu16or18)

```
git clone git@github.com:sbgisen/cube_petit_ros.git
cd cube_petit_ros/cube_petit_ros
catkin bt
source ~/.bashrc
```
## Quick Usage

### 1. bringup
On real robot
```
roslaunch cube_petit_bringup cube_petit_bringup.launch
```

On Gazebo9
```
roslaunch cube_petit_gazebo cube_petit_gazebo_pc.launch
```
### 2. slam

```
roslaunch cube_petit_navigation cube_petit_gmapping.launch 
```

### 3. navigation

```
roslaunch cube_petit_navigation cube_petit_naviagation.launch
```

---

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
