import spidev
from .icm20948_const import *

class IMU:
    def __init__(self):
        self.spi = spidev.SpiDev(0,0)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0b11
        
        accel_fs_sel = FSR.FSR_2g
        gyro_fs_sel = DPS.DPS_250
        self.accel_sensitivity = ACCEL_SENSITIVITY[accel_fs_sel]
        self.gyro_sensitivity = GYRO_SENSITIVITY[gyro_fs_sel]
        
        while not self.who_am_i():
            pass
        
        self.write_data(PWR_MGMT_1, 0x01)
        self.write_data(PWR_MGMT_2, 0x00)
        
        self.set_accel_config(ACCEL_FS_SEL = accel_fs_sel)
        self.set_gyro_config1(GYRO_FS_SEL = gyro_fs_sel)
        
    def __del__(self):
        self.spi.close()
        
    def write_data(self, reg, data):
        data_to_send = [0x7F & reg]
        if isinstance(data, list):
            data_to_send.extend(data)
        else:
            data_to_send.append(data)
        self.spi.writebytes(data_to_send)
    
    def read_data(self, reg):
        data_to_send = [0x80 | reg, 0x00]
        recv = self.spi.xfer(data_to_send)
        return recv[1:]
    
    def read_16bit(self, reg):
        high = self.read_data(reg)[0]
        low = self.read_data(reg + 1)[0]
        val = (high << 8) | low
        if val & 0x8000:
            val -= 65536
        return val
    
    # Set User bank 0 ~ 3
    def set_ub(self, ub):
        if ub < 0 or ub > 3:
            print("User bank can be set between 0 and 3.")
            return
        self.write_data(0x7F, 0 << 4)
        
    def who_am_i(self):
        self.set_ub(0)
        recv = self.read_data(WHO_AM_I)
        return 0xEA == recv[0]
    
    def set_gyro_config1(self, GYRO_DLPFCFG = 0b000, GYRO_FS_SEL = DPS.DPS_250, GYRO_FCHOICE = 0b0):
        self.set_ub(2)
        config = (GYRO_DLPFCFG << 3) | (GYRO_FS_SEL.value << 1) | GYRO_FCHOICE
        self.write_data(GYRO_CONFIG_1, config)
        
    def set_accel_config(self, ACCEL_DLPFCFG = 0b000, ACCEL_FS_SEL = FSR.FSR_2g, ACCEL_FCHOICE = 0b0):
        self.set_ub(2)
        config = (ACCEL_DLPFCFG << 3) | (ACCEL_FS_SEL.value << 1) | ACCEL_FCHOICE
        self.write_data(ACCEL_CONFIG, config)
        
    def get_accel(self):
        self.set_ub(0)
        ax = self.read_16bit(ACCEL_XOUT_H) / self.accel_sensitivity
        ay = self.read_16bit(ACCEL_YOUT_H) / self.accel_sensitivity
        az = self.read_16bit(ACCEL_ZOUT_H) / self.accel_sensitivity
        return [ax, ay, az]
    
    def get_gyro(self):
        self.set_ub(0)
        gx = self.read_16bit(GYRO_XOUT_H) / self.gyro_sensitivity
        gy = self.read_16bit(GYRO_YOUT_H) / self.gyro_sensitivity
        gz = self.read_16bit(GYRO_ZOUT_H) / self.gyro_sensitivity
        return [gx, gy, gz]

