# start


---

## burger_war/scripts/start.sh

~~~
#!/bin/bash

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node
roslaunch burger_war sim_robot_run.launch
~~~

## burger_war/launch/sim_robot_run.launch

~~~
<?xml version="1.0"?>
<launch>

<!-- Your robot control node run  red side-->
  <include file="$(find burger_war)/launch/your_burger.launch">
    <arg name="side" value="r" />
  </include>

<!-- enemy bot run  blue side-->
  <group ns="enemy_bot">
    <node pkg="burger_war" type="enemy.py" name="enemyRun" output="screen"/>
  </group>

</launch>
~~~

---

## burger_war/burger_war/launch/your_burger.launch

~~~
<?xml version="1.0"?>
<launch>
  <arg name="side" default="r"/> <!-- "b" = blue side, "r" = red side  -->

    <!-- You can change control program  -->

    <!-- sample program node -->
    <node pkg="burger_war" type="randomRun.py" name="randomRun" output="screen"/>

    <!-- END sample program node -->

    <!-- End of your space  -->
</launch>
~~~

---

## burger_war/burger_war/scripts/randomRun.py

~~~
#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic. 
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random

from geometry_msgs.msg import Twist


class RandomBot():
    def __init__(self, bot_name="NoName"):
        # bot name 
        self.name = bot_name
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

    def calcTwist(self):
        value = random.randint(1,1000)
        if value < 250:
            x = 0.2
            th = 0
        elif value < 500:
            x = -0.2
            th = 0
        elif value < 750:
            x = 0
            th = 1
        elif value < 1000:
            x = 0
            th = -1
        else:
            x = 0
            th = 0
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def strategy(self):
        r = rospy.Rate(1) # change speed 1fps

        target_speed = 0
        target_turn = 0
        control_speed = 0
        control_turn = 0

        while not rospy.is_shutdown():
            twist = self.calcTwist()
            print(twist)
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('random_run')
    bot = RandomBot('Random')
    bot.strategy()
~~~

---

## burger_war/burger_war/launch/sim_level_1_cheese.launch

~~~
<?xml version="1.0"?>
<launch>

<!-- Your robot control node run  red side-->
  <include file="$(find burger_war)/launch/your_burger.launch">
    <arg name="side" value="r" />
  </include>

<!-- enemy bot run  blue side-->
  <group ns="enemy_bot">
    <node pkg="burger_war" type="level_1_cheese.py" name="cheeseRun" output="screen"/>
  </group>

</launch>
~~~

---

## burger_war/burger_war/scripts/level_1_cheese.py

~~~
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# level_1_cheese.py
# write by yamaguchi takuya @dashimaki360
## GO and Back only


import rospy
import random

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

class CheeseBurger():
    def __init__(self, bot_name):
        # bot name 
        self.name = bot_name
        # robot state 'go' or 'back'
        self.state = 'back' 
        # robot wheel rot 
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0

        # speed [m/s]
        self.speed = 0.25

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odomCallback)
        self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

    def odomCallback(self, data):
        '''
        Dont use odom in this program now
        update robot pose in gazebo
        '''
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y

    def jointstateCallback(self, data):
        '''
        update wheel rotation num
        '''
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]

    def calcTwist(self):
        '''
        calc twist from self.state
        'go' -> self.speed,  'back' -> -self.speed
        '''
        if self.state == 'go':
            # set speed x axis
            x = self.speed
        elif self.state == 'back':
            # set speed x axis
            x = -1 * self.speed
        else:
            # error state
            x = 0
            rospy.logerr("SioBot state is invalid value %s", self.state)

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        return twist

    def calcState(self):
        '''
        update robot state 'go' or 'back'
        '''
        if self.state == 'go' and self.wheel_rot_r > 30:
            self.state = 'back'
        elif self.state == 'back' and self.wheel_rot_r < 5:
            self.state = 'go'

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        Go and Back loop forever
        '''
        r = rospy.Rate(5) # change speed 1fps

        while not rospy.is_shutdown():
            # update state from now state and wheel rotation
            self.calcState()
            # update twist
            twist = self.calcTwist()

            # publish twist topic
            self.vel_pub.publish(twist)

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('enemy')
    bot = CheeseBurger('cheese_burger')
    bot.strategy()
~~~

---
