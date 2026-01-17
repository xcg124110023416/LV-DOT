# 动态障碍物信息消息
# 用于规划器获取动态检测结果

std_msgs/Header header

# 障碍物数量
uint32 num

# 位置数组 (x, y, z 为中心坐标)
geometry_msgs/Vector3[] position

# 速度数组 (vx, vy, vz)
geometry_msgs/Vector3[] velocity

# 尺寸数组 (x_width, y_width, z_width 为完整尺寸)
geometry_msgs/Vector3[] size
