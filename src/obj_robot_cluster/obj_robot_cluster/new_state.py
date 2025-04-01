import numpy as np

class NewState():
    def __init__(self, lookahead_time, mpc_ts, close_to_target_rate = 0.5):
        self.lookahead_time = lookahead_time
        self.mpc_ts = mpc_ts
        self.close_to_target_rate = close_to_target_rate

    def get_target_point(self, old_traj, current_time, traj_time):
        target_time = current_time + self.lookahead_time
        look_ahead_idx = int((target_time - traj_time) / self.mpc_ts)
        trajectory_list = [old_traj[i:i+3] for i in range(0, len(old_traj), 3)]
        # print(f'lookahead_idx:{look_ahead_idx}')
        # print(f'traj_range:{len(trajectory_list)}')
        if look_ahead_idx < 0:
            look_ahead_idx = 0
        elif look_ahead_idx >= len(trajectory_list):
            look_ahead_idx = len(trajectory_list) - 1
        
        return (
                trajectory_list[0][0],
                trajectory_list[0][1]
            )
    
    def cal_new_state(self, current_pos, target):
        new_x = current_pos[0] + ((target[0]-current_pos[0]) * self.close_to_target_rate)
        new_y = current_pos[1] + ((target[1]-current_pos[1]) * self.close_to_target_rate)
        return (new_x, new_y)
    
    def get_new_current_state(self, old_traj, current_time, current_pos, traj_time):
        # print(f'check traj :{old_traj}')
        if self.lookahead_time is None or len(old_traj) == 0:
            new_state = current_pos
            # print('no update')
        else:
            target = self.get_target_point(old_traj,current_time, traj_time)
            new_state = self.cal_new_state(current_pos, target)
            # print('updated')
        return new_state