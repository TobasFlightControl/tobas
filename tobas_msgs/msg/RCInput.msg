std_msgs/Header header
float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
bool enable       # CH5: Enable Radio Control
                  # CH6: Reserved
uint8 mode        # CH7: Flight Mode
bool gpsw         # CH8: General Purpose Switch

uint8 MODE_ACROBAT = 0
uint8 MODE_STABILIZE = 1
uint8 MODE_LOITER = 2
