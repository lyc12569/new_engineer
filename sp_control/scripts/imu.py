#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import math
import numpy as np

# 初始化变量
gravity = [0, 0, 0]
gyro_offset = [0, 0, 0]
orientation = [0, 0, 0]
dt = 1/400

def calculate_gravity(accel_x,accel_y,accel_z,gyro_x, gyro_y, gyro_z):
    global gravity, gyro_offset, orientation

    # 计算陀螺仪旋转向量
    gyro_vector = [gyro_x * dt, gyro_y * dt, gyro_z * dt]
    gyro_matrix = euler_to_matrix(gyro_vector)
    orientation = matrix_to_euler(matrix_multiply(gyro_matrix, euler_to_matrix(orientation)))

    # 计算总加速度和重力加速度
    total_accel = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    gravity = [0, 0, -total_accel]
    gravity = matrix_multiply(euler_to_matrix(orientation), gravity)

# 将欧拉角转换为旋转矩阵
def euler_to_matrix(euler_angles):
    yaw, pitch, roll = euler_angles
    Rz = [[math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]]
    Ry = [[math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]]
    Rx = [[1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]]
    return matrix_multiply(Rz, matrix_multiply(Ry, Rx))

# 将旋转矩阵转换为欧拉角
def matrix_to_euler(R):
    sy = math.sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0])
    if sy > 1e-6:
        yaw = math.atan2(R[1][0], R[0][0])
        pitch = math.atan2(-R[2][0], sy)
        roll = math.atan2(R[2][1], R[2][2])
    else:
        yaw = math.atan2(-R[0][1], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        roll = 0
    return [yaw, pitch, roll]

# 计算矩阵乘积
def matrix_multiply(A, B):
    # return [[sum(a * b for a, b in zip(row_a, col_b))
    #          for col_b in zip(*B)]
    #         for row_a in A]
    return np.multiply(A,B) 
# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
imu = pipeline.create(dai.node.IMU)
xlinkOut = pipeline.create(dai.node.XLinkOut)

xlinkOut.setStreamName("imu")

# enable ACCELEROMETER_RAW at 500 hz rate
imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 500)
# enable GYROSCOPE_RAW at 400 hz rate
imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 400)
# it's recommended to set both setBatchReportThreshold and setMaxBatchReports to 20 when integrating in a pipeline with a lot of input/output connections
# above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
imu.setBatchReportThreshold(1)
# maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
# if lower or equal to batchReportThreshold then the sending is always blocking on device
# useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
imu.setMaxBatchReports(10)

# Link plugins IMU -> XLINK
imu.out.link(xlinkOut.input)

# Pipeline is defined, now we can connect to the device
with dai.Device(pipeline) as device:

    def timeDeltaToMilliS(delta) -> float:
        return delta.total_seconds()*1000

    # Output queue for imu bulk packets
    imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
    baseTs = None
    while True:
        imuData = imuQueue.get()  # blocking call, will wait until a new data has arrived

        imuPackets = imuData.packets
        for imuPacket in imuPackets:
            acceleroValues = imuPacket.acceleroMeter
            gyroValues = imuPacket.gyroscope

            acceleroTs = acceleroValues.getTimestampDevice()
            gyroTs = gyroValues.getTimestampDevice()
            if baseTs is None:
                baseTs = acceleroTs if acceleroTs < gyroTs else gyroTs
            acceleroTs = timeDeltaToMilliS(acceleroTs - baseTs)
            gyroTs = timeDeltaToMilliS(gyroTs - baseTs)

            imuF = "{:.06f}"
            tsF  = "{:.03f}"

            print(f"Accelerometer timestamp: {tsF.format(acceleroTs)} ms")
            print(f"Accelerometer [m/s^2]: x: {imuF.format(acceleroValues.x)} y: {imuF.format(acceleroValues.y)} z: {imuF.format(acceleroValues.z)}")
            print(f"Gyroscope timestamp: {tsF.format(gyroTs)} ms")
            print(f"Gyroscope [rad/s]: x: {imuF.format(gyroValues.x)} y: {imuF.format(gyroValues.y)} z: {imuF.format(gyroValues.z)} ")
            calculate_gravity(acceleroValues.x,acceleroValues.y,acceleroValues.z,gyroValues.x,gyroValues.y,gyroValues.z)
            print(gravity)
            # print("Gravity: ({:.2f}, {:.2f}, {:.2f})".format(*gravity))
            

        if cv2.waitKey(1) == ord('q'):
            break