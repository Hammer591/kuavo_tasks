#!/usr/bin/env python3
import rospy
import sys
import keyboard
import tty
import termios

from geometry_msgs.msg import Twist

# 获取键盘输入的函数（非阻塞）
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())  # 设置终端为原始模式
        ch = sys.stdin.read(1)  # 读取一个字符
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # 恢复终端设置
    print(ch, end = '',flush=True)
    return ch   
    
def move_robot(linear_speed, angular_speed):
    # 初始化 ROS 节点
    rospy.init_node('move_robot', anonymous=True)

    # 创建一个发布者，发布到/cmd_vel话题，消息类型为Twist
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # 设置循环频率
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # 创建一个 Twist 消息
        move_cmd = Twist()
		
        # 设置线速度和角速度
        direction = getch()
        if direction == 'w':
            move_cmd.linear.x = linear_speed
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
        elif direction == 'a':
            move_cmd.linear.x = 0
            move_cmd.linear.y = -linear_speed
            move_cmd.angular.z = 0
        elif direction == 'd':
            move_cmd.linear.x = 0
            move_cmd.linear.y = linear_speed
            move_cmd.angular.z = 0
        elif direction == 's':
            move_cmd.linear.x = -linear_speed
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
        elif direction == 'q':
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = angular_speed
        elif direction == 'e':
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = -angular_speed
        else:
            break
        #move_cmd.linear.x = linear_speed
        #move_cmd.angular.z = angular_speed
    # 发布消息
        pub.publish(move_cmd)
        #rospy.loginfo("Published velocity command: linear=%s, angular=%s", linear_speed, angular_speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 设置线速度和角速度
        linear_speed = 0.3  # 0.3 m/s
        angular_speed = 0.2  # 0.2 rad/s

        # 调用函数
        move_robot(linear_speed, angular_speed)
    except rospy.ROSInterruptException:
        pass
