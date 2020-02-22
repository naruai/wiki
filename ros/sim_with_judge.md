# sim_with_judge


---

## burger_war/scripts/sim_with_judge.sh

~~~
#!/bin/bash
set -e
set -x

# judge
# run judge server and visualize window
gnome-terminal -e "python3 judge/judgeServer.py"
gnome-terminal -e "python3 judge/visualizeWindow.py"

# init judge server for sim setting
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy

# robot
roslaunch burger_war setup_sim.launch
~~~

---

## burger_war/burger_war/launch/setup_sim.launch

~~~
<?xml version="1.0"?>

<launch>
<!-- make world -->
  <arg name="world_file"  default="$(find burger_war)/world/burger_field.world"/>
  <arg name="gui" default="true"/>
  <arg name="record" default="false"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="$(arg gui)" />
    <arg name="recording" value="$(arg record)" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>


<!-- your burger red side -->
  <!-- spawn the robot -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder $(find burger_war)/models/red_bot.urdf.xacro" />
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -x 0.0 -y -1.3 -z 0.0 -Y 1.57 -model red_bot -param robot_description">
  </node>
  
  <!-- speed limitter -->
  <node pkg="burger_war" type="speedLimitter.py" name="speed_limitter" output="screen">
  </node>


  <!-- ar reader -->
 <node pkg="aruco_ros" type="marker_publisher" name="aruco_marker_publisher">
        <param name="use_camera_info" value="False"/>
        <remap from="/image" to="image_raw" />
        <remap from="/aruco_marker_publisher/markers" to="target_id" />
 </node>

  <!-- send target ID to judge server -->
  <node pkg="burger_war" type="sendIdToJudge.py" name="send_id_to_judge" output="screen">
    <param name="judge_url" value="http://127.0.0.1:5000" />
    <param name="side" value="r" />
  </node>

<!-- enemy burger blue side -->
  <group ns="enemy_bot">
      <!-- spawn the robot -->
      <param name="robot_description" command="$(find xacro)/xacro --inorder $(find burger_war)/models/blue_bot.urdf.xacro" />
      <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-urdf -x 0.0 -y 1.3 -z 0.0 -Y -1.57 -model blue_bot -param robot_description">
      </node>
  
      <!-- speed limitter -->
      <node pkg="burger_war" type="speedLimitter.py" name="speed_limitter" output="screen">
      </node>

      <!-- ar reader -->
      <node pkg="aruco_ros" type="marker_publisher" name="aruco_marker_publisher">
          <param name="use_camera_info" value="False"/>
          <remap from="/image" to="image_raw" />
          <remap from="/enemy_bot/aruco_marker_publisher/markers" to="target_id" />
      </node>

      <!-- send target ID to judge server -->
      <node pkg="burger_war" type="sendIdToJudge.py" name="send_id_to_judge" output="screen">
        <param name="judge_url" value="http://127.0.0.1:5000" />
        <param name="side" value="b" />
      </node>

  </group>
</launch>
~~~

---
