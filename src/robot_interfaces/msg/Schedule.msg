uint32 robot_id                  # 机器人ID
geometry_msgs/Pose2D start_pose  # 起始位姿
geometry_msgs/Pose2D goal_pose   # 目标位姿
geometry_msgs/Point[] waypoints  # 路径点
float32[] timestamps            # 对应的时间戳
uint8 priority                  # 任务优先级
builtin_interfaces/Time stamp   # 时间戳
