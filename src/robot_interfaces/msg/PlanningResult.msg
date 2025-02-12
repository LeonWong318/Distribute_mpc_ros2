uint32 robot_id                        # 机器人ID
geometry_msgs/PoseArray planned_path   # 规划路径
float32[] velocities                   # 速度规划
float32[] timestamps                   # 时间戳
string planning_method                 # 规划方法
builtin_interfaces/Time stamp         # 时间戳