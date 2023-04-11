import os
import json
import os.path as osp
from glob import glob

import numpy as np
from tqdm import tqdm

from process.config import Config as cfg

os.makedirs(cfg.op_npy_pth, exist_ok=True)

for sequence in cfg.sequence_list:
    json_pth = osp.join(cfg.op_json_pth, sequence)
    json_fname_list = sorted(glob(osp.join(json_pth, '*.json')))
    
    pose_list = []
    for json_fname in tqdm(json_fname_list, desc='Converting json to npy', leave=False):
        with open(json_fname, 'r') as f:
            data = json.load(f)
            if len(data['people']) == 0:
                pose = np.zeros((25, 3))
            
            else:
                pose = np.array(data['people'][0]['pose_keypoints_2d']).reshape(25, 3)
            pose_list.append(pose)
        
    pose_list = np.stack(pose_list)
    out_fname = osp.join(cfg.op_npy_pth, sequence + '.npy')
    np.save(out_fname, pose_list)