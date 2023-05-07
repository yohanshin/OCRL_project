import os
import os.path as osp

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from process.config import Config as cfg
from process.sync_op import resample_data

op_mapper = {
    'lankle': 14, 'rankle': 11,
    'lknee': 13, 'rknee': 10,
    'lhip': 12, 'rhip': 9,
    'lshoulder': 5, 'rshoulder': 2,
}

height = 1.7

if __name__ == '__main__':
    for sequence in cfg.sequence_list:
        # Load data (frames, joints, 3)
        op = np.load(osp.join(cfg.sync_pth, sequence + '.npy'))
        op[..., 1] *= -1        # Flip Y-axis
        op_height = (op[:, :, 1].max(1) - op[:, :, 1].min(1))[:100].mean()
        scale = height / op_height
        
        # Set origin
        origin = op[:, [op_mapper[key] for key in ['lankle', 'rankle']]]
        origin = origin[origin[..., -1] > 0].mean(0)
        
        # Offset the origin ground
        op[..., :-1] -= origin[None, None, :-1]
        
        # Q1, Q2
        hip = op[:, [op_mapper[key] for key in ['lhip', 'rhip']]].mean(1)
        q1 = hip[:, 0] * scale      # Hip X-position
        q2 = hip[:, 1] * scale      # Hip Y-position
        
        # Q3, Q4, Q5
        shoulder = op[:, [op_mapper[key] for key in ['lshoulder', 'rshoulder']]].mean(1)
        knee = op[:, [op_mapper[key] for key in ['lknee', 'rknee']]].mean(1)
        ankle = op[:, [op_mapper[key] for key in ['lankle', 'rankle']]].mean(1)
        
        # Q3 (trunk angle)
        trunk = (shoulder - hip)[:, :-1]
        trunk /= np.linalg.norm(trunk, axis=1)[:, None]
        q3 = np.arctan2(trunk[:, 1], trunk[:, 0])      # Trunk angle
        
        # Q4 (Hip angle)
        thigh = (hip - knee)[:, :-1]
        thigh /= np.linalg.norm(thigh, axis=1)[:, None]
        thigh = np.arctan2(thigh[:, 1], thigh[:, 0])
        q4 = thigh - q3      # Thigh angle
        
        # Q5 (shank angle)
        shank = (knee - ankle)[..., :-1]
        shank /= np.linalg.norm(shank, axis=1)[:, None]
        shank = np.arctan2(shank[:, 1], shank[:, 0])
        q5 = np.pi + shank - thigh    # Shank angle
        
        # Q6-11 (Position of Head top, Knee, and Ankle)        
        q6 = shoulder[:, 0] * scale
        q7 = shoulder[:, 1] * scale
        q8 = knee[:, 0] * scale
        q9 = knee[:, 1] * scale
        q10 = ankle[:, 0] * scale
        q11 = ankle[:, 1] * scale
        
        # Get Mocap data
        mocap_pth = 'dataset/MSV_Optimal_Control_Data/IK_trial_2/Mocap'
        
        # Get knee angle
        knee = np.stack([np.load(osp.join(mocap_pth, leg + 'knee', leg + 'knee_angle_trimmed.npy')) for leg in ['l', 'r']])
        knee = knee.mean(0) / 180 * np.pi
        knee = resample_data(knee, cfg.mocap_fps, cfg.video_fps)
        knee = knee[:, 2]
        
        # Get hip angle
        hip = np.stack([np.load(osp.join(mocap_pth, leg + 'hip', leg + 'hip_angle_trimmed.npy')) for leg in ['l', 'r']])
        hip = hip.mean(0) / 180 * np.pi
        hip = resample_data(hip, cfg.mocap_fps, cfg.video_fps)
        hip = hip[:, 2]
        
        # Create DataFrame
        keys = [f'q{i}' for i in range(1, 12)]
        keys += ['knee', 'hip']
        df = pd.DataFrame(np.stack([globals()[key] for key in keys], axis=1), columns=keys)
        
        # Save to CSV
        df.to_csv(osp.join(cfg.sync_pth, sequence + '.csv'), index=False)
        