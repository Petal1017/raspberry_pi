import smbus
import time
import math

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimate_variance, initial_value):
        self.process_variance = process_variance  # 过程噪声的方差
        self.measurement_variance = measurement_variance  # 测量噪声的方差
        self.estimate_variance = estimate_variance  # 估计误差的方差
        self.estimate = initial_value  # 初始估计
        self.error_covariance = 1.0  # 初始误差协方差

    def update(self, measurement):
        # 预测步骤
        self.estimate_variance += self.process_variance
        
        # 计算卡尔曼增益
        kalman_gain = self.estimate_variance / (self.estimate_variance + self.measurement_variance)
        
        # 更新估计值
        self.estimate += kalman_gain * (measurement - self.estimate)
        
        # 更新误差协方差
        self.estimate_variance = (1 - kalman_gain) * self.estimate_variance

        return self.estimate

# MPU6050寄存器地址
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# 初始化MPU6050
def MPU_Init():
    # 写到寄存器 SMPLRT_DIV
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    
    # 写到寄存器 PWR_MGMT_1
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    
    # 写到寄存器 CONFIG
    bus.write_byte_data(Device_Address, CONFIG, 0)
    
    # 写到寄存器 GYRO_CONFIG
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    
    # 写到寄存器 INT_ENABLE
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

# 读取原始数据
def read_raw_data(addr):
    # 读取高8位
    high = bus.read_byte_data(Device_Address, addr)
    # 读取低8位
    low = bus.read_byte_data(Device_Address, addr+1)
    
    # 组合高8位和低8位
    value = ((high << 8) | low)
    
    # 转换为有符号整数
    if(value > 32768):
        value = value - 65536
    return value

# 卡尔曼滤波器类
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimate_variance, initial_value):
        self.process_variance = process_variance  # 过程噪声的方差
        self.measurement_variance = measurement_variance  # 测量噪声的方差
        self.estimate_variance = estimate_variance  # 估计误差的方差
        self.estimate = initial_value  # 初始估计
        self.error_covariance = 1.0  # 初始误差协方差

    def update(self, measurement):
        # 预测步骤
        self.estimate_variance += self.process_variance
        
        # 计算卡尔曼增益
        kalman_gain = self.estimate_variance / (self.estimate_variance + self.measurement_variance)
        
        # 更新估计值
        self.estimate += kalman_gain * (measurement - self.estimate)
        
        # 更新误差协方差
        self.estimate_variance = (1 - kalman_gain) * self.estimate_variance

        return self.estimate

# I2C地址
Device_Address = 0x68

# 打开I2C总线
bus = smbus.SMBus(1)

# 初始化MPU6050
MPU_Init()

# 初始化卡尔曼滤波器
kalman_roll = KalmanFilter(process_variance=0.1, measurement_variance=0.1, estimate_variance=0.1, initial_value=0.0)
kalman_pitch = KalmanFilter(process_variance=0.1, measurement_variance=0.1, estimate_variance=0.1, initial_value=0.0)

print("Reading Data from MPU6050")

while True:
    # 读取加速度计数据
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    
    # 读取陀螺仪数据
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_YOUT_H)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    
    # 转换为g值
    Ax = acc_x / 16384.0
    Ay = acc_y / 16384.0
    Az = acc_z / 16384.0
    
    # 转换为度/秒
    Gx = gyro_x / 131.0
    Gy = gyro_y / 131.0
    Gz = gyro_z / 131.0
    
    # 计算滚转角和俯仰角
    roll = math.atan2(Ay, Az) * 180 / math.pi
    pitch = math.atan2(-Ax, math.sqrt(Ay*Ay + Az*Az)) * 180 / math.pi

    # 应用卡尔曼滤波器
    roll_filtered = kalman_roll.update(roll)
    pitch_filtered = kalman_pitch.update(pitch)

    print(f"Filtered Roll: {roll_filtered:.2f}, Filtered Pitch: {pitch_filtered:.2f}")

    time.sleep(0.1)
