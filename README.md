# zzangdol_bringup

Last Update : 23.12.23   
Last Updated by : GeonhaPark ( Seunmul )

## zzangdol-ai-car bringup packages 

ROS bringup packages for zzangdol-ai-car, depends on ROS1.    
Bring up below devices and exectue nodes:
- RPLidar A1
- MCU firmware(zzangdol-core-v1)
- MyAHRS+
- cmd_vel converter node

  
## 1. Quick start

Below codes will execute cmd_vel_converter node, rosserial node,
rplidar, myahrs roslaunch file for zzangdol-ai-car.

```bash
 roslaunch zzangdol_bringup zzangdol_bring_all.launch usb_config:=false record:=false
```

- usbconfig **[required]** : usb_config selects ports, true or false
- record **[required]** : When record option is true, it records bag files

## 2. Detail info

#### src / cmd_vel_converter.cpp

- converter node which converts cmd_vel to cmd_vel_converted topic
- cmd_vel converted topic is specific control value which is compatible with zzangdol-ai-car motor driver.

Other launch files can execute with below commands

```bash
roslaunch zzangdol_bringup [below_launch_file]
```

### launch files

#### launch / zzangdol_bring_all.launch

- main bringup file for oprating zzangdol_ai_car

#### launch / zzangdol_bring_minimal.launch

- essential bringup file for testing senser data

#### launch / zzangdol_core.launch

- cmd_vel conveter, mcu nodes (rosserial) launch file

#### launch / zzangdol_lidar.launch

- lidar launch file
- needs rplidar_ros package <https://github.com/robopeak/rplidar_ros>

#### launch / zzangdol_myahrs.launch

- myahrs launch file
- needs myahrs_driver <https://github.com/robotpilot/myahrs_driver>

#### launch / zzangdol_state_publisher.launch

- zzangdol_car tf topic publisher node launch file

### codes

#### src / cmd_vel_converter.cpp

- Converter node which converts cmd_vel and cmd_vel_ehco topic to firmware accessible data
- **cmd_vel** topic is command from move_base(Ros Navigation Stack)
- **cmd_vel_converted** topic is specific control value which is compatible with zzangdol-ai-car mcu nodes(Arduino Mega, 8bits integer type needs). Converts cmd_vel topic to cmd_vel_converted
- **cmd_vel_echo** topic is value which MCU is used to control DC Motor
- **cmd_vel_echo_converted** topic presents cmd_vel topic value from cmd_vel_echo(which depends on zzangdol-ai-car HW)
- float <-> int8
