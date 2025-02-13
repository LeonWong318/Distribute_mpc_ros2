std_msgs/Header header
int32 ego_robot_id         # 接收机器人的ID
float32[] states           # 扁平化的其他机器人状态数组，包含当前状态和预测状态
int32 state_dim           # 每个状态的维度
int32 horizon             # 预测时域长度
int32 num_others         # 其他机器人的数量