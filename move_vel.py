#!/usr/bin/env python3
import rospy
import sys
import select
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import mode_schedule  # 导入 mode_schedule 消息类型

# 非阻塞获取键盘输入的函数
def get_input_non_blocking():
    # 使用 select 检测是否有输入
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        return sys.stdin.readline().strip()  # 读取一行输入并去除换行符
    return None  # 如果没有输入，返回 None

def move_vel():
    # 初始化 ROS 节点（只需调用一次）
    rospy.init_node('move_vel_gait_publisher', anonymous=True)  # 修改节点名称
    
    # 创建一个发布者，发布到 /cmd_vel 话题，消息类型为 Twist
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # 创建一个发布者，发布到 /your_robot_name_mpc_mode_schedule 话题，消息类型为 mode_schedule
    gait_pub = rospy.Publisher('/humanoid_mpc_mode_schedule', mode_schedule, queue_size=1,latch=True)
    
    # 设置循环频率
    rate = rospy.Rate(10)
    
    # 定义模式字符串到整数的映射
    mode_mapping = {
        "STANCE": 3,
        "LCONTACT": 1,
        "RCONTACT": 2
    }
    
    # 建立步态映射
    gaitmap_ = {
        "stance": {
            "modeSequence": ["STANCE"],
            "switchingTimes": [0.0, 1000.0]
        },
        "trot": {
            "modeSequence": ["LCONTACT", "RCONTACT"],
            "switchingTimes": [0.0, 0.45, 0.9]
        },
        "walk": {
            "modeSequence": ["LCONTACT", "STANCE", "RCONTACT", "STANCE"],
            "switchingTimes": [0.0, 0.45, 0.6, 1.05, 1.2]
        },
        "quick_trot": {
            "modeSequence": ["LCONTACT", "RCONTACT"],
            "switchingTimes": [0.0, 0.3, 0.6]
        }
    }   
    
    # 初始化速度
    linear_speed = 0.0  # 初始速度为 0
    gait = 'walk'     # 初始步态为站立
    input_buffer = ""   # 用于存储输入的数字

    print("Enter a speed value (e.g., 0.5, -0.2) and press Enter to confirm.")
    print("Press 'q' to quit.")

    while not rospy.is_shutdown():
        # 非阻塞获取键盘输入
        user_input = get_input_non_blocking()
        
        if user_input is not None:  # 如果有输入
            if user_input == 'q':  # 退出程序
                break
            else:  # 处理速度输入
                try:
                    # 将输入转换为浮点数并设置为线速度
                    linear_speed = float(user_input)
                    print(f"Set linear speed to: {linear_speed} m/s")
                    if linear_speed == 0.0:
                        gait = 'stance'
                    else:
                        gait = 'walk'
                    
                    # 创建一个 mode_schedule 消息
                    msg = mode_schedule()
                    # 将模式字符串转换为整数
                    msg.modeSequence = [mode_mapping[mode] for mode in gaitmap_[gait]["modeSequence"]]
                    msg.eventTimes = gaitmap_[gait]["switchingTimes"]  # 设置切换时间
                    # 发布消息

                    gait_pub.publish(msg)      # 发布步态消息
                except ValueError:
                    print("Invalid input! Please enter a number.")
        
        # 创建一个 Twist 消息
        move_cmd = Twist()
        move_cmd.linear.x = linear_speed  # 设置线速度
        move_cmd.angular.z = 0.0         # 角速度保持为 0
        vel_pub.publish(move_cmd)  # 发布速度消息
        rate.sleep()

    # 停止机器人
    move_cmd = Twist()
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    vel_pub.publish(move_cmd)
    print("\nStopping the robot.")

if __name__ == '__main__':
    try:
        # 设置标准输入为非阻塞模式
        import tty
        import termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(sys.stdin.fileno())  # 设置终端为 cbreak 模式
            move_vel()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # 恢复终端设置
    except rospy.ROSInterruptException:
        pass
