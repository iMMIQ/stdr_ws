# Copyright: www.corvin.cn ROS小课堂 2016-2021
# Author: corvin
# Description:
     本代码仓库主要存储STDR仿真器相关测试代码，主要用于学习如何通过
   代码来进行ROS实验，由于该仿真器是二维平面移动，因此主要用于仿真两
   轮差速驱动的小车,当然自己建全向移动的小车也是可以的。
     目前代码仓库中包含的软件包及其介绍如下,随着在STDR中的实验不断增
   多，软件包也会逐渐增加：
     (1).stdr_simulator:STDR仿真器软件的实现源码,可以根据需要来修改源
        码增加额外功能。
     (2).ultrasonic_obstacle_avoidance:用于利用自带的仿真小车上超声波
        数据进行避障仿真实验。
     (3).teleop_twist_keyboard:使用键盘可以遥控仿真小车四处移动。
     (4).stdr_gmapping:在stdr中进行gmapping进行仿真建图的测试代码。
     (5).stdr_move_base:在stdr中调用move_base软件包，配置相关参数。
     (6).stdr_amcl:在stdr中进行amcl定位和自动导航的测试代码。
     (7).stdr_hector_mapping:使用hector_mapping完成地图构建。
     (8).stdr_navigation:最上层的代码,通过调用move_base,amcl等实现自动导航.

# Usage:
   0.在使用该仿真代码前，首先需要运行工作区根目录下scripts目录中的install.sh脚本,
     该脚本只需要在刚下载代码时执行一次即可，后面就不用再执行了:
     sudo ./setup_env.sh
     等待该命令执行完成即可.

   1.加载提供的其他地图,并加载一个机器人:
     (1).加载frieburg地图:
         roslaunch stdr_launchers frieburg_map_robot_gui.launch
     (2).加载hospital_section地图:
         roslaunch stdr_launchers hospital_map_robot_gui.launch
     (3).加载mines地图:
         roslaunch stdr_launchers mines_map_robot_gui.launch
     (4).加载robocup地图:
         roslaunch stdr_launchers robocup_map_robot_gui.launch
     (5).加载rooms地图:
         roslaunch stdr_launchers rooms_map_robot_gui.launch
     (6).加载nowall_rooms地图:
         roslaunch stdr_launchers nowall_rooms_map_robot_gui.launch

   2.需要测试各SLAM算法，可以查看如下各操作：
     (1):需要测试gmapping建图，执行如下命令：
         roslaunch stdr_gmapping stdr_gmapping_keyboard.launch

         需要使用键盘来遥控机器人四处走动，执行以下命令：
         rosrun teleop_twist_keyboard teleop_twist_keyboard.py

     (2):需要测试hector_mapping建图,执行如下命令：
         roslaunch stdr_hector_mapping stdr_hector_mapping.launch

         需要使用键盘来遥控机器人四处走动，执行以下命令：
         rosrun teleop_twist_keyboard teleop_twist_keyboard.py

     (3):想要实现定点巡航仿真的话，执行如下命令：
         roslaunch stdr_navigation patrol_nav.launch

   3.需要测试超声波避障策略：
     (1):最基础的根据前面三个超声波来进行避障移动，执行以下命令：
         roslaunch ultrasonic_obstacle_avoidance ultrasonic_avoidance.launch


# History:
    20180523: init this code project.
    20180601: add stdr_hector_mapping package to test hector.
    20180806: 新增可以定点巡航的测试代码patrol_nav.launch,patrol_nav.py.
    20180903: 新增install.sh,用于安装一些ros软件包,这样才能正常编译和使用该仿真.
    20181220: 新增了可以加载其他地图启动的launch文件,位于stdr_launch文件夹中.
    20210326: 删除无用的launch文件，删除frame中的/，修改膨胀层系数.
