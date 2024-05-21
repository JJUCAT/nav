uint16 STATUS_INVALID = 0
uint16 STATUS_RECORDING_POLYGON = 1
uint16 STATUS_RECORDING_PATH = 2
uint16 STATUS_RECORDING_POINT = 3

uint16 mode
uint16 status           # 1: polygon, 2: path, 3: point
geometry_msgs/PoseStamped[] poses_stamped
