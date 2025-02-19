發那科手臂執行 : 開啟六個視窗

**開啟D435節點

1. ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true

**到piper_ws工作空間下執行目標偵測package

2. cd ~/piper_ws & source install/setup.bash

ros2 run transform_example yolov8_detect

**到fanuc_ws工作空間下啟動crx10ia_l moveit包

3. cd ~/fanuc_ws & source install/setup.bash

ros2 launch crx10ia_l_moveit_config demo.launch.py

**將相機座標轉換至Fanuc世界座標

4. ros2 run tf2_ros static_transform_publisher 1.5 0.0 0.7 3.14 0 0 fanuc_world camera_link

** 將偵測到的object座標從相機座標轉到Fanuc基座座標

5. ros2 run fanuc_joint_sql moveit_fram_to_world


整合4. 5. =  ros2 launch fanuc_joint_sql  demo.launch.py


** 啟動moveit座標規劃 移動至object座標

6. ros2 run fanuc_joint_sql moveit_goal_setting

** 下達啟動指令，使moveit規劃庫執行

 ros2 service call /trigger_plan std_srvs/srv/Trigger {}

** 監聽手臂末端座標及姿態值

ros2 run tf2_ros tf2_echo fanuc_world link_6

** 啟動監聽joint 傳送到SQL

ros2 run fanuc_joint_sql joint_states_listener_sql.py 








