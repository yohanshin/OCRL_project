# Predicting Biomechanical Outcomes of Sit-to-Stand with Direct Collocation

## Quick Starter Guide
The control part was implemented in MATLAB using fmincon solver from the OptimTraj toolbox. To run the direct collocation, go to the folder ```control``` and run the file ```main_sts_tracking.m```.

## Data
Data includes
- ```data.csv``` contains reference trajectory generated using OpenPose.
- ```data_truth.csv``` contains ground truth motion capture for kinematics validation.

## Dynamics Modeling
Dynamical model was developed with the aid of the library ```SimPy``` in Python (see ```modeling/sts_5dof_dyn.py``` for more detail). The outputted model was then converted to MATLAB codes (see ```dynamics.m``` and ```dynamics_n.m```) to perform simulation (see ```main_dyn_sim.m```) and trajectory optimization (see ```main_sts_tracking.m```).

## Model Parameters
Parameters were set up based on the seminal work by de Leva (see ```model_params.m```).

## Reference Trajectory Generation
Codes for obtaining the reference trajectory for tracking can be found in the folder ```process```.

## Results
Results can be found in the folder ```results```.