std_msgs/Header header
float64 roll      # CH1: [-1, 1]
float64 pitch     # CH2: [-1, 1]
float64 throttle  # CH3: [-1, 1]
float64 yaw       # CH4: [-1, 1]
bool enable       # CH5: Enable Radio Control
bool kill         # CH6: Kill Switch
uint8 mode        # CH7: Flight Mode
bool sub_mode     # CH8: Sub Flight Mode
bool[8] gpsw      # CH9-16: General Purpose Switch
