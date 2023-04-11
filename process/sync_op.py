import os, sys
import os.path as osp
from glob import glob

import json
import torch
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import torch.nn.functional as F
from scipy.signal import find_peaks

from process.config import Config as cfg


def read_mocap_data(fname):
    pd_data = pd.read_csv(fname, header=0, index_col=[0, 1], skiprows=(0, 1, 2, 4, 5, 6))
    markers = pd_data.columns.tolist()
    marker_idxs = [i for i in range(len(markers)) if 'RPS1' in markers[i]]
    data = pd_data.iloc[:, marker_idxs].values
    data = resample_data(data, cfg.mocap_fps, cfg.video_fps)
    return data


def read_openpose_data(fname):
    height = 1.7
    
    openpose = np.load(fname)
    
    # Scale to meters
    op_height = openpose[:, :, 1].max() - openpose[:, :, 1].min()
    scale = height / op_height
    
    data = openpose[:, 8]
    data[:, :2] *= scale
    data[:, 1] *= -1 # Flip y-axis
    return data


def resample_data(data, org_fps, trg_fps):
    """Resample data to the target framerate
    """
    l = int(data.shape[0] / org_fps * trg_fps)
    to_numpy = False
    
    if isinstance(data, np.ndarray):
        to_numpy = True
        data = torch.from_numpy(data)

    data = F.interpolate(data.T.unsqueeze(0), l, mode='linear').squeeze(0).T
        
    if to_numpy:
        data = data.numpy()
    
    return data

def find_hopping_idx(data, is_front=True, th=10):
    """Find hopping index
    For the front hopping, find the first peak of the acceleration.
    For the end hopping, find the last peak of the acceleration.
    """
    
    data = data - data[:500].mean(0, keepdims=True)      # Remove offset
    idxs, _ = find_peaks(data, height=th)
    trg_idx = 0 if is_front else -1
    idx = idxs[trg_idx]
    return idx

def find_syncing_idx(data1, data2, idx1, idx2, output_path, sequence, pad=30, winsize=60, searchsize_half=60, stride=1, viz=False):
    """Find syncing point of two data; data1 and data2.
    
    If the output sync_idx is positive, meaning that data2 begins earlier than data1
    """
    
    data1 = data1 - data1[:500].mean(0, keepdims=True)      # Remove offset
    data2 = data2 - data2[:500].mean(0, keepdims=True)      # Remove offset
    
    # Make sure window size is not larger than actual start of the hopping sequence
    pad = min(pad, min(idx1, idx2))
    
    base_idx = idx2 - idx1
    data1_crop = data1[idx1-pad:idx1+winsize+pad]
    data1_crop = data1_crop - data1_crop.mean(0, keepdims=True)
    
    rmse_list = []
    search_space = list(range(max(-idx1+pad, base_idx - searchsize_half), base_idx + searchsize_half+1))
    for start in search_space:
        end = start + winsize
        data2_crop = data2[idx1+start-pad:idx1+end+pad]
        data2_crop = data2_crop - data2_crop.mean(0, keepdims=True)
        rmse = ((data1_crop - data2_crop) ** 2).mean() ** 0.5
        rmse_list.append(rmse)
    
    min_idx = np.argmin(np.array(rmse_list))
    sync_idx = search_space[min_idx]
    
    if viz:
        os.makedirs(output_path, exist_ok=True)
        fig, ax = plt.subplots(2, 1, figsize=(4.5, 7))
        
        # Draw RMSE
        ax[0].plot(search_space, rmse_list, linewidth=3, color='tab:orange')
        ax[0].set_xlabel('Time Index', fontsize=12)
        ax[0].set_ylabel('RMSE (m)', fontsize=12)
        ax[0].spines['top'].set_visible(False)
        ax[0].spines['right'].set_visible(False)
        
        if sync_idx > 0:
            # If sync_idx > 0, IMU starts earlier than Mocap
            data2_crop = data2[sync_idx:][idx1-pad:idx1+winsize+pad]
        else:
            # If sync_idx <= 0, Mocap starts earlier (or same) than IMU
            data2_crop = data2[idx1-pad:idx1+winsize+pad]
            data1_crop = data1[-sync_idx:][idx1-pad:idx1+winsize+pad]
        
        ax[1].plot(data1_crop, color='tab:orange', label='Mocap')
        ax[1].plot(data2_crop, color='tab:blue', label='OpenPose')
        ax[1].legend()
        ax[1].set_xlabel('Time Index', fontsize=12)
        ax[1].set_ylabel('Pelvis Height (m)', fontsize=12)
        ax[1].spines['top'].set_visible(False)
        ax[1].spines['right'].set_visible(False)
        fig.tight_layout(pad=5.0)
        plt.subplots_adjust(left=0.2, bottom=0.15)
        plt.savefig(osp.join(output_path, f'{sequence}.png'))
        plt.close('all')
    
    return sync_idx


os.makedirs(cfg.sync_pth, exist_ok=True)

for sequence in cfg.sequence_list:
    # Load OpenPose data
    openpose_fname = osp.join(cfg.op_npy_pth, sequence + '.npy')
    oc_pelv = read_openpose_data(openpose_fname)
    
    # Load Mocap data
    trial = sequence.split('_')[0][-1]
    mocap_fname = glob(osp.join(cfg.mocap_pth, f'*trial_{trial}*output.csv'))[0]
    mc_pelv = read_mocap_data(mocap_fname)
    
    # Extract hopping sequence
    oc_idx = find_hopping_idx(oc_pelv[:, 1], is_front=True, th=0.1)
    mc_idx = find_hopping_idx(mc_pelv[:, 1], is_front=True, th=0.1)
    
    # Sync two data
    sync_idx = find_syncing_idx(mc_pelv[:, 1], 
                                oc_pelv[:, 1], 
                                mc_idx, oc_idx, 
                                osp.join(cfg.sync_pth, 'Visualization'), 
                                sequence, 
                                viz=True)
    
    openpose = np.load(openpose_fname)
    if sync_idx > 0:
        openpose = openpose[sync_idx:]
    elif sync_idx < 0:
        openpose = np.concatenate((np.stack([openpose[0] for i in range(-sync_idx)]), openpose))
    
    # Deleting or padding the back of the data to match the mocap data length
    if openpose.shape[0] > mc_pelv.shape[0]:
        openpose = openpose[:mc_pelv.shape[0]]
    elif openpose.shape[0] < mc_pelv.shape[0]:
        l_diff = mc_pelv.shape[0] - openpose.shape[0]
        openpose = np.concatenate((openpose, np.stack([openpose[-1] for i in range(l_diff)])))
    
    np.save(osp.join(cfg.sync_pth, sequence + '.npy'), openpose)