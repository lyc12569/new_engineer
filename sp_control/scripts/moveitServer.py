#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 导入基本ros和moveit库
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
import numpy as np
import math
import tf
from tf.transformations import *
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion

# robotiq-85夹爪库调用，没用使用ros控制夹爪
#from real.robotiq_gripper import RobotiqGripper

# # 负责接收深度学习传入的抓取位姿，并反馈抓取结果
# from ur_planning.srv import grasp_pose,grasp_poseRequest,grasp_poseResponse

class MoveIt_Control:
    # 初始化程序
    def __init__(self, is_use_gripper=False):
        # Init ros config
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_control_server', anonymous=False)
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        self.arm.set_goal_joint_tolerance(0.001)
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.01)

        self.end_effector_link = self.arm.get_end_effector_link()
        # 设置机械臂基座的参考系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)

        # 设置最大规划时间和是否允许重新规划
        self.arm.set_planning_time(5)
        self.arm.allow_replanning(True)
        self.arm.set_planner_id("RRTstar")

        # 设置允许的最大速度和加速度（范围：0~1）
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)

        # 机械臂初始姿态
        self.go_home()
        # 发布场景
        # self.set_scene()  # set table
        #self.arm.set_workspace([-2,-2,0,2,2,2])  #[minx miny minz maxx maxy maxz]
        # 测试专用
        # self.testRobot()
        # 抓取服务端，负责接收抓取位姿并执行运动
        # server = rospy.Service("moveit_grasp",grasp_pose,self.grasp_callback )
        # rospy.spin()

    def close(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # 测试程序用
    def testRobot(self):
        # try:
        print("Test for robot...")
        # self.go_home()
        self.move_j([-0.1, 0.2, 0, 0, 0.5, 0.5,0.5],a=0.5,v=0.5)
        # rospy.sleep(2)
        
        # self.move_p([-0.04,0.225, 0.799, 3.1408791693424214, 0.19548416616730288, 3.1389621847279248])
        # rospy.sleep(5)
        # self.set_constraints()
        # self.move_l([0.4, 0.1, 0.4, -np.pi, 0, 0] )
        # rospy.sleep(2)
        # except:
        #     print("Test fail! ")

    # 在机械臂下方添加一个table，使得机械臂只能够在上半空间进行规划和运动
    # 避免碰撞到下方的桌子等其他物体
    def set_scene(self):
        ## set table
        self.scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)
        table_id = 'table'
        self.scene.remove_world_object(table_id)
        rospy.sleep(1)
        table_size = [2, 2, 0.01]
        table_pose = PoseStamped()
        table_pose.header.frame_id = self.reference_frame
        table_pose.pose.position.x = 0.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = -table_size[2]/2 -0.02
        table_pose.pose.orientation.w = 1.0
        self.scene.add_box(table_id, table_pose, table_size)
        self.setColor(table_id, 0.5, 0.5, 0.5, 1.0)
        self.sendColors()

    # 关节规划，输入6个关节角度（单位：弧度）
    def move_j(self, joint_configuration=None,a=1,v=1):
        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        if joint_configuration==None:
            joint_configuration = [0, -1.5707, 0, -1.5707, 0, 0]
        self.arm.set_start_state_to_current_state();    
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        self.arm.set_joint_value_target(joint_configuration)
        rospy.loginfo("move_j:"+str(joint_configuration))
        self.arm.go()
        # rospy.sleep(1)

    # 空间规划，输入xyzRPY
    def move_p(self, tool_configuration=None,a=1,v=1):
        if tool_configuration==None:
            tool_configuration = [0.3,0,0.3,0,-np.pi/2,0]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.reference_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position.x = tool_configuration[0]
        target_pose.pose.position.y = tool_configuration[1]
        target_pose.pose.position.z = tool_configuration[2]
        q = quaternion_from_euler(tool_configuration[3],tool_configuration[4],tool_configuration[5])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        print(target_pose)

        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        rospy.loginfo("move_p:" + str(tool_configuration))
        success = self.arm.go(wait=True)
        print(success)
        # traj = self.arm.plan()
        # print(traj)
        # self.arm.execute(traj)
        # rospy.sleep(1)

    # 空间直线运动，输入(x,y,z,R,P,Y,x2,y2,z2,R2,...)
    # 默认仅执行一个点位，可以选择传入多个点位
    def move_l(self, tool_configuration,waypoints_number=1,a=0.5,v=0.5):
        if tool_configuration==None:
            tool_configuration = [0.3,0,0.3,0,-np.pi/2,0]
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)

        # 设置路点
        waypoints = []
        for i in range(waypoints_number):
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.reference_frame
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = tool_configuration[6*i+0]
            target_pose.pose.position.y = tool_configuration[6*i+1]
            target_pose.pose.position.z = tool_configuration[6*i+2]
            q = quaternion_from_euler(tool_configuration[6*i+3],tool_configuration[6*i+4],tool_configuration[6*i+5])
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            waypoints.append(target_pose.pose)
        rospy.loginfo("move_l:" + str(tool_configuration))
        self.arm.set_start_state_to_current_state()
        fraction = 0.0  # 路径规划覆盖率
        maxtries = 100  # 最大尝试规划次数
        attempts = 0  # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                waypoints,  # waypoint poses，路点列表
                0.001,  # eef_step，终端步进值
                0.00,  # jump_threshold，跳跃阈值
                True)  # avoid_collisions，避障规划
            attempts += 1
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) +
                " success after " + str(maxtries) + " attempts.")
        rospy.sleep(1)

    def move_c(self,pose_via,tool_configuration,k_acc=1,k_vel=1,r=0,mode=0):
        pass

    def go_home(self,a=1,v=1):
        self.arm.set_max_acceleration_scaling_factor(a)
        self.arm.set_max_velocity_scaling_factor(v)
        # “up”为自定义姿态，你可以使用“home”或者其他姿态
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)

    def setColor(self, name, r, g, b, a=0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()
        # 需要设置规划场景是否有差异
        p.is_diff = True
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)
    
    ## robotiq85夹爪相关的函数，可以不使用
    ## ----------------------------------------------------
    def log_gripper_info(self):
        rospy.loginfo("Pos: "+str(self.gripper.get_current_position()   ))
    def close_gripper(self,speed=255,force=255):
        # position: int[0-255], speed: int[0-255], force: int[0-255]
        self.gripper.move_and_wait_for_pos(255, speed, force)
        # self.log_gripper_info()
        # print("gripper had closed!")
        rospy.sleep(1.2)
    def open_gripper(self,speed=255,force=255):
        # position: int[0-255], speed: int[0-255], force: int[0-255]
        self.gripper.move_and_wait_for_pos(0, speed, force)
        # self.log_gripper_info()
        # print("gripper had opened!")
        rospy.sleep(1.2)
    def check_grasp(self):
        # if the robot grasp object ,then the gripper is not open
        return self.get_current_tool_pos()>5
    def get_current_tool_pos(self):
        return self.gripper.get_current_position() 
  


controller_values_initial = PoseStamped()
# controller_values_initial = Pose()
droll = 0
dpitch = 0
dyaw = 0
dx = 0
dy = 0
dz = 0
empty = PoseStamped()
# empty = Pose()
print(empty == controller_values_initial)

def joint_value_callback(data,moveit_server):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data) 
    global controller_values_initial,droll,dpitch,dyaw,dx,dy,dz
    if controller_values_initial == empty:
        controller_values_initial = data
    else:
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        roll_last, pitch_last,yaw_last = euler_from_quaternion([controller_values_initial.pose.orientation.x, controller_values_initial.pose.orientation.y,
                                                                controller_values_initial.pose.orientation.z, controller_values_initial.pose.orientation.w])
        droll = roll - roll_last
        dpitch = pitch - pitch_last
        dyaw = yaw - yaw_last
        dx = data.pose.position.x - controller_values_initial.pose.position.x
        dy = data.pose.position.y - controller_values_initial.pose.position.y
        dz = data.pose.position.z - controller_values_initial.pose.position.z
        print(dx)
        controller_values_initial = data
        if droll != 0 or dpitch != 0 or dyaw != 0 or dx != 0 or dy != 0 or dz != 0:
            current_values = moveit_server.arm.get_current_joint_values()
            next_values = [-0.01,0.01,0.01,0.01,0.01,0.01,0.01]
            # joint1
            if -0.20 <= current_values[0] + dz <= -0.01:
                next_values[0] = current_values[0] + dz
            elif current_values[0] + dz < -0.20:
                next_values[0] = -0.20
            #joint2
            if 0 <= current_values[1] - dy <= 0.866:
                next_values[1] = current_values[1] - dy
            elif current_values[1] - dy > 0.866:
                next_values[1] = 0.866
            #joint4
            if -0.185 <= current_values[3] +dx <= 0.185:
                next_values[3] = current_values[3] + dx
            elif current_values[3] +dx > 0.185:
                next_values[3] = 0.158
            elif current_values[3] +dx < -0.185:
                next_values[3] = -0.185
            #joint5
            if 0 <= current_values[4] + dyaw <= np.pi:
                next_values[4] = current_values[4] + dyaw
            elif current_values[4] + dyaw > np.pi:
                next_values[4] = np.pi
            #joint6
            if -np.pi/2 <= current_values[5] +droll <= np.pi/2:
                next_values[5] =  current_values[5] +droll
            elif current_values[5] +droll > np.pi/2:
                next_values[5] = np.pi/2
            elif current_values[5] +droll < -np.pi/2:
                next_values[5] = -np.pi/2    
            #joint7
            if -np.pi/2 <= current_values[6] +dpitch <= np.pi/2:
                next_values[6] =  current_values[6] +dpitch
            elif current_values[6] +dpitch > np.pi/2:
                next_values[6] = np.pi/2
            elif current_values[6] +dpitch < -np.pi/2:
                next_values[6] = -np.pi/2     
            
            print(next_values)
            moveit_server.move_j(next_values)    

def plan_callback(data,moveit_server):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data) 
    global controller_values_initial,droll,dpitch,dyaw,dx,dy,dz
    if controller_values_initial == empty:
        controller_values_initial = data
    else:
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        roll_last, pitch_last,yaw_last = euler_from_quaternion([controller_values_initial.pose.orientation.x, controller_values_initial.pose.orientation.y,
                                                                controller_values_initial.pose.orientation.z, controller_values_initial.pose.orientation.w])
        droll = roll - roll_last
        dpitch = pitch - pitch_last
        dyaw = yaw - yaw_last
        dx = data.pose.position.x - controller_values_initial.pose.position.x
        dy = data.pose.position.y - controller_values_initial.pose.position.y
        dz = data.pose.position.z - controller_values_initial.pose.position.z
        controller_values_initial = data
        if droll != 0 or dpitch != 0 or dyaw != 0 or dx != 0 or dy != 0 or dz != 0:
            current_pose = moveit_server.arm.get_current_pose()
            print(current_pose)
            # (croll, cpitch, cyaw) = euler_from_quaternion([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])
            # next_pose = [-0.01,0.01,0.01,0.01,0.01,0.01]
            # # print(current_pose)
            # next_pose[0] = current_pose.pose.position.x + dx
            # next_pose[1] = current_pose.pose.position.y + dy
            # next_pose[2] = current_pose.pose.position.z + dz
            # next_pose[3] = croll + droll
            # next_pose[4] = cpitch + dpitch
            # next_pose[5] = cyaw + dyaw
            # # print(next_pose)
            # moveit_server.move_p(next_pose)   

def plan_callback(data,moveit_server):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data) 
    global controller_values_initial,droll,dpitch,dyaw,dx,dy,dz
    if controller_values_initial == empty:
        controller_values_initial = data
    else:
        (roll, pitch, yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        roll_last, pitch_last,yaw_last = euler_from_quaternion([controller_values_initial.pose.orientation.x, controller_values_initial.pose.orientation.y,
                                                                controller_values_initial.pose.orientation.z, controller_values_initial.pose.orientation.w])
        droll = roll - roll_last
        dpitch = pitch - pitch_last
        dyaw = yaw - yaw_last
        dx = data.pose.position.x - controller_values_initial.pose.position.x
        dy = data.pose.position.y - controller_values_initial.pose.position.y
        dz = data.pose.position.z - controller_values_initial.pose.position.z
        controller_values_initial = data
        if droll != 0 or dpitch != 0 or dyaw != 0 or dx != 0 or dy != 0 or dz != 0:
            current_pose = moveit_server.arm.get_current_pose()
            print(current_pose)
        
if __name__ =="__main__":
    moveit_server = MoveIt_Control(is_use_gripper=False)
    current_values = moveit_server.arm.get_current_joint_values()
    # current_pose = moveit_server.arm.get_current_pose()
    # print(current_pose)
    # print(moveit_server.arm.get_current_rpy())
    # print(current_values)
    # rospy.init_node("listener_pose")
    # #3.实例化 订阅者 对象
    # sub = rospy.Subscriber("/ORB_SLAM3/pose",PoseStamped,joint_value_callback,moveit_server,queue_size=1)
    # rospy.spin()
 
    

