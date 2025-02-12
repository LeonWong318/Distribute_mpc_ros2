uint32 robot_id                    # 发生碰撞风险的机器人ID
uint32[] potential_collision_ids   # 可能发生碰撞的其他机器人ID
geometry_msgs/Point collision_point # 潜在碰撞点
float32 time_to_collision         # 预计碰撞时间
uint8 severity                    # 严重程度
builtin_interfaces/Time stamp     # 时间戳