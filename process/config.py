

class Config:
    sequence_list = ['trial1_camera1', 'trial1_camera2', 'trial2_camera1', 'trial2_camera2']
    
    video_pth = 'dataset/RGB_Videos'
    mocap_pth = 'dataset/MSV_Optimal_Control_Data'
    sync_pth = 'dataset/OpenPose_Synced'
    op_base_pth = 'dataset/OpenPose'
    op_json_pth = 'dataset/OpenPose/output_json'
    op_npy_pth = 'dataset/OpenPose/output_npy'
    
    mocap_fps = 100
    video_fps = 30