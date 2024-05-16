std_msgs/Header header
tobas_msgs/RCInputError error
float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
uint8 mode        # CH5: Flight Mode
bool e_stop       # CH7: Emergency Stop
bool gpsw         # CH8: General Purpose Switch

uint8 MODE_PROGRAM = 0
uint8 MODE_STABILIZE = 1
uint8 MODE_ACROBAT = 2
