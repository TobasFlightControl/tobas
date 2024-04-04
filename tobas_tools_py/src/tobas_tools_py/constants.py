import os.path as osp

SERVO_RAIL_SIZE = 14
MIN_PWM = 1000
MAX_PWM = 2000
ARM_THROTTLE = 0.1

RCIN_ROLL = 0  # CH1
RCIN_PITCH = 1  # CH2
RCIN_THRUST = 2  # CH3
RCIN_YAW = 3  # CH4
RCIN_MODE = 4  # CH5
RCIN_ESTOP = 6  # CH7
RCIN_GPSW = 7  # CH8

CONFIG_PATH = osp.expanduser(f"~/.config/tobas/config.ini")
