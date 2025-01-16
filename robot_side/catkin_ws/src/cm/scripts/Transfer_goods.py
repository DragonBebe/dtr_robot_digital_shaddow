#!/usr/bin/env python3
# Software License Agreement (BSD License)
# 
# This script controls a motor to turn following a unit-pulse signal. And monitor the response.
import threading, rospy, Board, time
from cm.msg import msg_cm as RosJointState
# from cm.msg import WorkRange
# import argparse
import time
import numpy as np
from math import sqrt
from ArmIK.InverseKinematics import *
import random

#random.seed(50)

#the robotic arm moves according to the angle calculated by inverse kinematics
ik = IK('arm')
#set the length of link
l1 = ik.l1 + 0.75
l4 = ik.l4 - 0.15
ik.setLinkLength(L1=l1, L4=l4)


class ArmIK:
    servo3Range = (0, 1000.0, 0, 240.0)  # pulse width， angle
    servo4Range = (0, 1000.0, 0, 240.0)
    servo5Range = (0, 1000.0, 0, 240.0)
    servo6Range = (0, 1000.0, 0, 240.0)

    def __init__(self):
        self.setServoRange()

    def setServoRange(self, servo3_Range=servo3Range, servo4_Range=servo4Range, servo5_Range=servo5Range,
                      servo6_Range=servo6Range):
        # adapt to different servo
        self.servo3Range = servo3_Range
        self.servo4Range = servo4_Range
        self.servo5Range = servo5_Range
        self.servo6Range = servo6_Range
        self.servo3Param = (self.servo3Range[1] - self.servo3Range[0]) / (self.servo3Range[3] - self.servo3Range[2])
        self.servo4Param = (self.servo4Range[1] - self.servo4Range[0]) / (self.servo4Range[3] - self.servo4Range[2])
        self.servo5Param = (self.servo5Range[1] - self.servo5Range[0]) / (self.servo5Range[3] - self.servo5Range[2])
        self.servo6Param = (self.servo6Range[1] - self.servo6Range[0]) / (self.servo6Range[3] - self.servo6Range[2])

    def transformAngelAdaptArm(self, theta3, theta4, theta5, theta6):
        # convert the angle calculated by inverse kinematics into pulse width corresponding to servo
        servo3 = int(round(theta3 * self.servo3Param + (self.servo3Range[1] + self.servo3Range[0]) / 2))
        if servo3 > self.servo3Range[1] or servo3 < self.servo3Range[0] + 60:
            logger.info('servo3(%s)exceed the range(%s, %s)', servo3, self.servo3Range[0] + 60, self.servo3Range[1])
            return False

        servo4 = int(round(theta4 * self.servo4Param + (self.servo4Range[1] + self.servo4Range[0]) / 2))
        if servo4 > self.servo4Range[1] or servo4 < self.servo4Range[0]:
            logger.info('servo4(%s)exceed the range(%s, %s)', servo4, self.servo4Range[0], self.servo4Range[1])
            return False

        servo5 = int(round((self.servo5Range[1] + self.servo5Range[0]) / 2 - (90.0 - theta5) * self.servo5Param))
        if servo5 > ((self.servo5Range[1] + self.servo5Range[0]) / 2 + 90 * self.servo5Param) or servo5 < (
                (self.servo5Range[1] + self.servo5Range[0]) / 2 - 90 * self.servo5Param):
            logger.info('servo5(%s)exceed the range(%s, %s)', servo5, self.servo5Range[0], self.servo5Range[1])
            return False

        if theta6 < -(self.servo6Range[3] - self.servo6Range[2]) / 2:
            servo6 = int(
                round(((self.servo6Range[3] - self.servo6Range[2]) / 2 + (90 + (180 + theta6))) * self.servo6Param))
        else:
            servo6 = int(round(((self.servo6Range[3] - self.servo6Range[2]) / 2 - (90 - theta6)) * self.servo6Param))
        if servo6 > self.servo6Range[1] or servo6 < self.servo6Range[0]:
            logger.info('servo6(%s)exceed the range(%s, %s)', servo6, self.servo6Range[0], self.servo6Range[1])
            return False

        return {"servo3": servo3, "servo4": servo4, "servo5": servo5, "servo6": servo6}

    # def servosMove(self, servos, movetime=None):
    #     # drive NO. 3, 4, 5 and 6 servo to rotate
    #     time.sleep(0.02)
    #     if movetime is None:
    #         max_d = 0
    #         for i in range(0, 4):
    #             d = abs(getBusServoPulse(i + 3) - servos[i])
    #             if d > max_d:
    #                 max_d = d
    #         movetime = int(max_d * 4)
    #     setBusServoPulse(3, servos[0], movetime)
    #     setBusServoPulse(4, servos[1], movetime)
    #     setBusServoPulse(5, servos[2], movetime)
    #     setBusServoPulse(6, servos[3], movetime)
    #
    #     return movetime

    def setPitchRange(self, coordinate_data, alpha1, alpha2, da=1):
        # The given coordinate is coordinate_data and pitch angle range is alpha1 and alpha2. Automatically search the appropriate solution within the range
        # If there is no solution, return False. Otherwise, return corresponding servo angle and pitch angle.
        # The coordinate in cm is passed in as tuple, for example (0, 5, 10)
        # da is increased angle in each pitch angle traverse
        x, y, z = coordinate_data
        if alpha1 >= alpha2:
            da = -da
        for alpha in np.arange(alpha1, alpha2, da):  # traverse to find solution
            result = ik.getRotationAngle((x, y, z), alpha)
            if result:
                theta3, theta4, theta5, theta6 = result['theta3'], result['theta4'], result['theta5'], result['theta6']
                servos = self.transformAngelAdaptArm(theta3, theta4, theta5, theta6)
                if servos != False:
                    return servos, alpha

        return False

    # def setPitchRangeMoving(self, coordinate_data, alpha, alpha1, alpha2, movetime=None):
    #     # The given coordinate is coordinate_data, pitch angle is alpha, and pitch angle range is alpha1 and alpha2. Automatically search the appropriate solution within the range. And rotate to the target position.
    #     # If there is no solution, return False. Otherwise return servo angle, pitch angle and running time.
    #     # The coordinate in cm is passed in as tuple, for example (0, 5, 10)
    #     # alpha is the given pitch angle
    #     # alpha1 and alpha2 are pitch angle range
    #     # movetime is servo rotation time in ms. If the time is not given, calculate automatically.
    #     x, y, z = coordinate_data
    #     result1 = self.setPitchRange((x, y, z), alpha, alpha1)
    #     result2 = self.setPitchRange((x, y, z), alpha, alpha2)
    #     if result1 != False:
    #         data = result1
    #         if result2 != False:
    #             if abs(result2[1] - alpha) < abs(result1[1] - alpha):
    #                 data = result2
    #     else:
    #         if result2 != False:
    #             data = result2
    #         else:
    #             return False
    #     servos, alpha = data[0], data[1]
    #
    #     movetime = self.servosMove((servos["servo3"], servos["servo4"], servos["servo5"], servos["servo6"]), movetime)
    #
    #     return servos, alpha, movetime

    # 更完整的电机计算方式，可以指定俯仰角alpha，根据给定的俯仰角范围进行计算：（alpha1， alpha2）
    def setPitchRangeServosInfo(self, coordinate_data, alpha, alpha1, alpha2):
        # The given coordinate is coordinate_data, pitch angle is alpha, and pitch angle range is alpha1 and alpha2. Automatically search the appropriate solution within the range. And rotate to the target position.
        # If there is no solution, return False. Otherwise return servo angle, pitch angle and running time.
        # The coordinate in cm is passed in as tuple, for example (0, 5, 10)
        # alpha is the given pitch angle
        # alpha1 and alpha2 are pitch angle range
        # movetime is servo rotation time in ms. If the time is not given, calculate automatically.
        x, y, z = coordinate_data
        result1 = self.setPitchRange((x, y, z), alpha, alpha1)
        result2 = self.setPitchRange((x, y, z), alpha, alpha2)
        if result1 != False:
            data = result1
            if result2 != False:
                if abs(result2[1] - alpha) < abs(result1[1] - alpha):
                    data = result2
        else:
            if result2 != False:
                data = result2
            else:
                return False
        servos, alpha = data[0], data[1]

        return servos, alpha


class CMDataPublisher:
    def __init__(self, node, io_block_flag: list, freq=10):
        self.node = node
        rate = self.node.get_param('~rate', freq)
        self.r = rospy.Rate(rate)
        self.io_block_flag = io_block_flag

        self.msg = RosJointState()
        self.msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'r_joint']
        self.msg.header.frame_id = 'not_relervant'
        self.msg.position = [0]*6
        self.msg.temperature = [0]*6
        self.msg.voltage = [0]*6

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/condition_monitoring', RosJointState, queue_size=50)
        rospy.loginfo("Starting Joint State Publisher at " + str(rate) + "Hz")


    def safe_read(self, monitored_motor: int):
        ''' Read the position safely. This function verifies if the IO is occupied before perform the reading operation.
        If IO is blocked, it will waits until it is released. During the reading operation, it will block the IO.        
        '''
        # Check if the io is blocked:
        while self.io_block_flag[0]:
            print('Thread_CM: Waiting for the IO to be released!')
            pass

        # Block the IO and perform the reading action.
        self.io_block_flag[0] = True
        # Get CM data.
        motor_idx = monitored_motor-1
        self.msg.position[motor_idx] = Board.getBusServoPulse(motor_idx+1) # Position
        self.msg.temperature[motor_idx] = Board.getBusServoTemp(motor_idx+1) # Temperature
        self.msg.voltage[motor_idx] = Board.getBusServoVin(motor_idx+1) # Voltage
        # Release the IO
        self.io_block_flag[0] = False


    def get_and_pub_cm_data(self):             
        # Log current time.
        self.msg.header.stamp = rospy.Time.now()
        # Get CM data.
        for monitored_motor in range(1, 7):
            self.safe_read(monitored_motor)
        # Publish the data.
        self.joint_states_pub.publish(self.msg)        


class ControlMotor:
    def __init__(self, node, io_block_flag: list):
        self.node = node
        self.io_block_flag = io_block_flag

        # Prepare initial values of the msg.
        self.msg = RosJointState()
        self.msg.name = ['Target value']
        self.msg.header.frame_id = 'not_relervant'
        self.msg.position = [0, 0, 0, 0, 0, 0]
        self.msg.temperature = [0, 0, 0, 0, 0, 0]
        self.msg.voltage = [0]

        self.monitor_pos_pub = rospy.Publisher('/position_monitoring', RosJointState, queue_size=1)


    def safe_control_motor(self, target_value: int, duration: int, monitored_motor: int):
        ''' Send the control command to a given motor safely. It verifies the IO is not occupied before sending the control command.
        During the sending operation, it will block the IO.
        '''
        # Check if the io is blocked:
        while self.io_block_flag[0]:
            print('Thread_Control: Waiting for the IO to be released!')
            pass

        # Block the IO and perform the reading action.
        self.io_block_flag[0] = True
        # Set target value.
        Board.setBusServoPulse(monitored_motor, target_value, duration)
        # Release the IO
        self.io_block_flag[0] = False


    def send_and_pub_control_signal(self, trajectory: list, duration_list: list):       
        # Log the current time.
        self.msg.header.stamp = rospy.Time.now()
        
        # Loop over the motors.
        for monitored_motor in range(1, 7):
            motor_idx = monitored_motor - 1
            target_value = trajectory[motor_idx]
            duration = duration_list[motor_idx]
            self.safe_control_motor(target_value, duration, monitored_motor)            
        # Sleep for 2 seconds. The time needed for the robot to finish one trajectory.
        time.sleep(2)
                
        # Publish the control command per trajectory.        
        self.msg.position = trajectory
        self.msg.temperature = duration_list
        self.monitor_pos_pub.publish(self.msg)
        # Log the information.
        rospy.loginfo('Publish control command: Position target: {}, Duration: {}ms'.format(self.msg.position, self.msg.temperature))       


def node_condition_monitoring(node, io_block_flag, freq=100):
    cm_data_publisher = CMDataPublisher(node, io_block_flag, freq)
    while not rospy.is_shutdown():
        cm_data_publisher.get_and_pub_cm_data()
        cm_data_publisher.r.sleep()

def node_control_robot(node, io_block_flag: list, trajectories=[[500, 500, 500, 500, 500, 500]], durations_lists=[[1000, 1000, 1000, 1000, 1000, 1000]]):
    # Initialize ros node.
    robot_controller = ControlMotor(node, io_block_flag)
    # Sleep for 5 seconds. Time needed to start the listener on the PC side.
    time.sleep(5)    
    # Loop over the trajectories. Send the control signals.
    for trajectory, duration_list in zip(trajectories, durations_lists):
        robot_controller.send_and_pub_control_signal(trajectory, duration_list)
  
# def publish_work_range(work_range_pub, work_range):
#    msg = WorkRange()
#    msg.x = [coord[0] for coord in work_range]
#    msg.y = [coord[1] for coord in work_range]
#    msg.z = [coord[2] for coord in work_range]
#    work_range_pub.publish(msg)


if __name__ == '__main__':
    """
    模拟Transfer good的形式，把方块从指定的抓取工作空间空间（关闭gripper），移动到放置空间（打开gripper）
    open_gripper = 200， close_gripper = 500
    需要设计系列动作实现完整的移动货物的功能：
    0. 起始状态：钳子打开，初始位置在（0， 0， 15）
    1. 开始Pick：移动到指定位置
    2. Pick：移动到指定位置以后钳子关闭
    3. 开始Place：钳子关闭以后移动到指定Place位置
    4. Place: 移动到指定位置以后，钳子打开
    5. 回到初始位置（0， 0， 15）
    """
    AK = ArmIK()

    # Initial position and nbr_goods
    nbr_goods = 3
    movetime = 2000
    # 俯仰角,(alpha, alpha1, alpha2): alpha代表希望gripper和水平线的夹角，alpha1和alpha2代表求解的搜索范围
    angleOrient = (0, -180, 0)
    # 随机工作范围设定，单位为cm
    PickR_Px = [-10, 10]
    PickR_Py = [8, 28]
    PickR_Pz = [4, 6]

    # 随机设定gripper位置
    Pick_range = [
        (int(round(random.uniform(PickR_Px[0], PickR_Px[1]))),
         int(round(random.uniform(PickR_Py[0], PickR_Py[1]))),
         int(round(random.uniform(PickR_Pz[0], PickR_Pz[1]))))
        for _ in range(nbr_goods)
    ]

    PlaceR_Px = [-30, -10]
    PlaceR_Py = [-10, 10]
    PlaceR_Pz = [15, 20]

    # 随机设定gripper位置
    Place_range = [
        (int(round(random.uniform(PlaceR_Px[0], PlaceR_Px[1]))),
         int(round(random.uniform(PlaceR_Py[0], PlaceR_Py[1]))),
         int(round(random.uniform(PlaceR_Pz[0], PlaceR_Pz[1]))))
        for _ in range(nbr_goods)
    ]

    trajectories = []

    open_gripper = 200
    close_gripper = 500

    # 通过逆运动学计算coordinate对应的舵机脉宽
    for i in range(nbr_goods):
        # 起始状态：钳子打开，初始位置在（0， 0， 15）
        initial_pos = (10, 0, 15)
        servos, alpha = AK.setPitchRangeServosInfo(initial_pos, angleOrient[0], angleOrient[1], angleOrient[2])
        trajectory = [
            open_gripper, 500,  # 固定的脉宽值
            servos['servo3'],
            servos['servo4'],
            servos['servo5'],
            servos['servo6']
        ]
        trajectories.append(trajectory)

        # 开始Pick：移动到指定位置
        servos, alpha = AK.setPitchRangeServosInfo(Pick_range[i], angleOrient[0], angleOrient[1], angleOrient[2])
        trajectory = [
            open_gripper, 500,  # 固定的脉宽值
            servos['servo3'],
            servos['servo4'],
            servos['servo5'],
            servos['servo6']
        ]
        trajectories.append(trajectory)

        # Pick：移动到指定位置以后钳子关闭
        trajectory = [
            close_gripper, 500,  # 固定的脉宽值
            servos['servo3'],
            servos['servo4'],
            servos['servo5'],
            servos['servo6']
        ]
        trajectories.append(trajectory)

        # 开始Place：钳子关闭以后移动到指定Place位置
        servos, alpha = AK.setPitchRangeServosInfo(Place_range[i], angleOrient[0], angleOrient[1], angleOrient[2])
        trajectory = [
            close_gripper, 500,  # 固定的脉宽值
            servos['servo3'],
            servos['servo4'],
            servos['servo5'],
            servos['servo6']
        ]
        trajectories.append(trajectory)

        # Place：移动到指定位置以后，钳子打开
        trajectory = [
            open_gripper, 500,  # 固定的脉宽值
            servos['servo3'],
            servos['servo4'],
            servos['servo5'],
            servos['servo6']
        ]
        trajectories.append(trajectory)

        # 回到初始位置（0， 0， 15）
        servos, alpha = AK.setPitchRangeServosInfo(initial_pos, angleOrient[0], angleOrient[1], angleOrient[2])
        trajectory = [
            open_gripper, 500,  # 固定的脉宽值
            servos['servo3'],
            servos['servo4'],
            servos['servo5'],
            servos['servo6']
        ]
        trajectories.append(trajectory)

    durations_lists = [[2000, 2000, 2000, 2000, 2000, 2000] for _ in range(len(trajectories))]

    # 定义io块标志
    io_block_flag = [False]

    try:
        print("Work space: ")
        # 在主线程中初始化ROS节点
        rospy.init_node('node_test_motor_position', anonymous=True)

        # 发布工作范围的信息
        # work_range_pub = rospy.Publisher('/work_range', WorkRange, queue_size=10)

        # 创建两个线程
        monitoring_freq = 10
        thread1 = threading.Thread(target=node_condition_monitoring, args=(rospy, io_block_flag, monitoring_freq))
        thread2 = threading.Thread(target=node_control_robot,
                                   args=(rospy, io_block_flag, trajectories, durations_lists))

        # 启动线程
        thread1.start()
        thread2.start()

        # 等待两个线程完成
        thread1.join()
        thread2.join()

    except rospy.ROSInterruptException:
        pass

    print("Transfer range: ", Pick_range, Place_range)
