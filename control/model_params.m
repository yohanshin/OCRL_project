% name: model_params.m
% description: Initialize parameters
% author: Vu Phan
% date: 2023/04/15

% Segment mass
m  = 60;                    % (kg) - total body mass
m2 = 2*m*14.78/100;         % (kg) - thighs mass
m3 = 2*m*(4.81 + 1.29)/100; % (kg) - shanks mass (actually, shanks + feet mass)
m1 = m - m2 - m3;           % (kg) - trunk mass (actually, torso + head + arms)

% Segment length
L1 = 0.8585;            % (m) - trunk length - to be measured on Mikayla
L2 = 0.3685;            % (m) - thigh length - to be measured on Mikayla
L3 = 0.4323 + 0.0386;   % (m) - shank length - to be measured on Mikayla

% Distance to segment CoM
r1 = L1*41.51/100; % (m) - trunk CoM from the hip
r2 = L2*36.12/100; % (m) - thigh CoM from the knee
r3 = L3*44.16/100; % (m) - shank CoM from the ankle

% Inertia
k1 = L1*35.7/100; % (m) - trunk gyration radius
k2 = L2*36.9/100; % (m) - thigh gyration radius
k3 = L3*27.1/100; % (m) - shank gyration radius
I1 = m1*k1^2; % (kg/m^2) - trunk moment of inertia
I2 = m2*k2^2; % (kg/m^2) - thigh moment of inertia
I3 = m3*k3^2; % (kg/m^2) - shank moment of inertia

% Gravitational acceleration
g = 9.81; % (m/s^2) - on Earth


% Foot contact
ground_height = -0.18; % better to model as a spring 
K_foot = 2*m*g/(0.05)^2;
b_foot = 0.7;
mu_foot = 1.0;

% Seat contact
seat_height = 0.45; % better to model as a spring too
K_hip = 2*m*g/(0.05)^2;
b_hip = 0.7;
mu_hip = 1.0;



