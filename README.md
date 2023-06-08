# zzangdol_core_v1 
## zzangdol-ai-car firmware-v1, bringup package
ros wrapper bringup codes for zzangdol-ai-car

## 1. Quick start

Below codes will execute cmd_vel_converter node, rosserial node,
rplidar, myahrs roslaunch file for zzangdol-ai-car.


```bash
roslaunch zzangdol_bringup zzangdol_bringup_all.launch
```

## 2. Package Explain

Other launch files can execute with below commands

```bash
roslaunch zzangdol_bringup [below launch file]
```

### launch files

#### launch / zzangdol_bring_all.launch

- main bringup file for oprating zzangdol_ai_car

#### launch / zzangdol_bring_minimal.launch

- essential bringup file for testing senser data

#### launch / zzangdol_core.launch

- mcu (cmd_vel conveter + rosserial) launch file

#### launch / zzangdol_lidar.launch

- lidar launch file

#### launch / zzangdol_myahrs.launch

- myahrs launch file

#### launch / zzangdol_state_publisher.launch

- zzangdol_ai_car HW tf topic publisher node launch file


### codes

#### src / cmd_vel_converter.cpp

- converter node which converts cmd_vel to cmd_vel_converted topic
- cmd_vel converted topic is specific control value which is compatible with zzangdol-ai-car motor driver.
