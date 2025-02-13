geometry_msgs/Pose2D pose         # 机器人位姿
geometry_msgs/Twist velocity      # 线速度和角速度
bool is_idle                     # 空闲状态
string current_task              # 当前任务
uint32 robot_id                  # 机器人ID
builtin_interfaces/Time stamp    # 时间戳
geometry_msgs/PoseArray predicted_poses     #预测的未来状态序列