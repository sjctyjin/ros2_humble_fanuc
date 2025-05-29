# 發那科手臂執行 :

## **1. 到 fanuc_ws 工作空間下啟動 crx10ia_l 包**
```bash
cd ~/fanuc_ws & source install/setup.bash

ros2 launch crx_description view_robot.launch.py robot_type:=crx10ia_l
```


## **2. 下達抓取放啟動指令，使 curobo 規劃庫執行**
```bash
ros2 run curobo_piper curobo_pick_and_place_mpc
```
### Window端啟動Fanuc Api 將IP 指向192.168.1.100
```bash
Fanuc_Control_241029\Fanuc_Control\bin\Debug\Fanuc_Control.exe
先在Fanuc_Control_241029\Fanuc_Control\Connect_info.json 中 修改Robot ip及SQL ip
```

## **監聽手臂末端座標及姿態值**
```bash
ros2 run tf2_ros tf2_echo fanuc_world link_6
```

