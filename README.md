# 發那科手臂執行 : 開啟六個視窗

## **1. 開啟 D435 節點**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
```

## **2. 到 piper_ws 工作空間下執行目標偵測 package**
```bash
cd ~/piper_ws & source install/setup.bash
ros2 run transform_example yolov8_detect
```

## **3. 到 fanuc_ws 工作空間下啟動 crx10ia_l moveit 包**
```bash
cd ~/fanuc_ws & source install/setup.bash
ros2 launch crx10ia_l_moveit_config demo.launch.py
```

## **4. 將相機座標轉換至 Fanuc 世界座標**
```bash
ros2 run tf2_ros static_transform_publisher 1.5 0.0 0.7 3.14 0 0 fanuc_world camera_link
```

## **5. 將偵測到的 object 座標從相機座標轉到 Fanuc 基座座標**
```bash
ros2 run fanuc_joint_sql moveit_fram_to_world
```
### **整合 4、5**
```bash
ros2 launch fanuc_joint_sql demo.launch.py
```

## **6. 啟動 moveit 座標規劃，移動至 object 座標**
```bash
ros2 run fanuc_joint_sql moveit_goal_setting
```

## **7. 下達啟動指令，使 moveit 規劃庫執行**
```bash
ros2 service call /trigger_plan std_srvs/srv/Trigger {}
```

## **8. 監聽手臂末端座標及姿態值**
```bash
ros2 run tf2_ros tf2_echo fanuc_world link_6
```

## **啟動監聽 joint 傳送到 SQL**
```bash
ros2 run fanuc_joint_sql joint_states_listener_sql.py
```
