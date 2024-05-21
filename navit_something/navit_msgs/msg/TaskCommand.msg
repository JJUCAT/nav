uint16 CMD_DEFAULT = 0  # default command
uint16 CMD_POLYGON = 1  # 执行多边形任务
uint16 CMD_PATH = 2     # 执行路径任务
uint16 CMD_POINT = 3    # 执行点任务
uint16 CMD_CHARGE = 4   # 执行充电任务
uint16 CMD_ABORT_CHARGE = 5   # 执行充电任务
uint16 CMD_MuiltPolygons_WITH_AUTODOCK = 6 # 执行多区域任务
uint16 cmd
uint16 repeat_times     # 执行次数
string[] seq            # 任务序列               