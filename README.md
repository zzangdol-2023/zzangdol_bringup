# zzangdol_core_v1 
## zzangdol-ai-car firmware-v1, bringup package
ros wrapper bringup codes for zzangdol-ai-car

## 1. Quick start

Below codes will execute cmd_vel_converter node, rosserial node,
rplidar, myahrs roslaunch file for zzangdol-ai-car.


```bash
roslaunch zzangdol_bringup zzangdol_bringup.launch
```

## 2. Package Explain

Other launch files can execute with below commands

```bash
roslaunch zzangdol_bringup [below launch file]
```

### launch files

#### launch / zzangdol_bringup.launch

- main bringup file

#### launch / zzangdol_cmd_vel_converter.launch

- cmd_vel converter launch file

#### launch / zzangdol_core.launch

- mcu (cmd_vel conveter + rosserial) launch file

#### launch / zzangdol_lidar.launch

- lidar launch file

#### launch / zzangdol_myahrs.launch

- myahrs launch file

#### launch / zzangdol_odom_tf.launch

- odom_tf (odom_tf - base_footprint tf topic launch) file

### codes

#### src / cmd_vel_converter.cpp

- converter node which converts cmd_vel to cmd_vel_converted topic
- cmd_vel converted topic is specific control value which is compatible with zzangdol-ai-car motor driver.

#### src / odom_tf_publisher_node.py

- odom_tf publisher node
- In our projects, used mcu is arduino mega, wich doesn't have FPU.
- Because of that, we cannot calculate exact value of odom_tf. so we used high-level node which publish odom_tf.
- This node's role :
  - 1. publish odom_tf which link odom_tf and base_footprint
  - 2. publish odom topic
