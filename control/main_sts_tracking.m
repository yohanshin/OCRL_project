% name: main_sts_tracking.m
% description: Perform tracking with direct collocation
% author: Vu Phan
% date: 2023/04/15

% FILES IN THIS PROJECT
% sts_5dof_dyn.py - derivation of the continuous dynamics
% dynamics.m - continuous dynamics of the model derived from PyDy
% dynamics_rk4.m - discrete dynamics of the model using vanilla RK4
% model_params.m - parameters of the model
% get_A.m and get_B.m - A and B matrices with finite difference
% main_dyn_sym.m - simulation of the dynamics with RK4

clear 
close all
clc

video_name = 'tracking_anime_12';
tracking_start = 380;
tracking_stop = 420;

%% Add OptimTraj toolbox
addpath('OptimTraj-master/OptimTraj-master')  

%% Get tracking data from OpenPose
xref = readmatrix('data.csv');
xref = xref(tracking_start:tracking_stop, :);
xref = xref';
xref(5, :) = xref(5, :) - pi; % BECAUSE OF OUR DATA

xpos = readmatrix('data_pos.csv');
xpos = xpos(tracking_start:tracking_stop, :);
xpos = xpos(:, [6, 7, 8, 9, 10, 11]);
xpos = xpos';

N = size(xref, 2); % number of samples
dt = 1/30.0;
duration = N*dt;
tgrid = 0:dt:duration;
tgrid = tgrid(1:end-1);

%% Init
model_params; % model parameters
dt = 1.0/30.0; % simulation settings

% Initial state variable
Q1 = 0.0;
Q2 = 0.8;
Q3 = 60.0*pi/180.0;
Q4 = 90.0*pi/180.0;
Q5 = -90.0*pi/180.0;
V1 = 0;
V2 = 0;
V3 = 0;
V4 = 0;
V5 = 0;
x0 = [Q1; Q2; Q3; Q4; Q5; V1; V2; V3; V4; V5];

% Initial control input
Fx1 = 0;
Fy1 = 600;
Fx2 = 0;
Fy2 = 0;
tau1 = 0;
tau2 = 0;
u0 = [Fx1; Fy1; Fx2; Fy2; tau1; tau2];

% Kinemtatics constraints (based on OpenSim model)
x_low = [-0.5; 0; 0.0*pi/180.0; -30.0*pi/180.0; -130.0*pi/180.0; -inf; -inf; -inf; -inf; -inf];
x_upp = [0.5; 1; 180.0*pi/180.0; 130*pi/180.0; 10.0*pi/180.0; inf; inf; inf; inf; inf];
% x_low = [-0.5; 0; -inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf];
% x_upp = [0.5; 1; inf; inf; inf; inf; inf; inf; inf; inf];

% Dynamics constraints
u_low = [-200; 0; -200; 0; -500; -500];
u_upp = [200; 700; 200; 700; 500; 500];

% Others
nx = length(x0); % number of state variables
nu = length(u0); % number of control inputs

%% Direct collocation
% Dynamics and cost settings
problem.func.dynamics = @(t, x, u)(dynamics_n(x, u)); % dynamics
% problem.func.pathObj = @(t, x, u)(quadratic_cost(x, u, xref)); % cost function
problem.func.pathObj = @(t, x, u)(quadratic_cost(x, u, xref, xpos)); % cost function

% Bounds settings 
% problem.bounds.initialTime.low = 0;
% problem.bounds.initialTime.upp = 0;
% problem.bounds.finalTime.low = duration;
% problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = xref(:, 1); % init state
problem.bounds.initialState.upp = xref(:, 1);
problem.bounds.finalState.low = xref(:, end); % final state
problem.bounds.finalState.upp = xref(:, end);

problem.bounds.state.low = x_low; % x_L <= x <= x_U
problem.bounds.state.upp = x_upp;
problem.bounds.control.low = u_low; % u_L <= u <= u_U
problem.bounds.control.upp = u_upp;

% Initial guess settings
problem.guess.time = tgrid;
% problem.guess.state = repmat(xref(:, 1), 1, length(tgrid));
% problem.guess.control = repmat(u0, 1, length(tgrid));
problem.guess.state = xref; % use xref as the initial guess
problem.guess.control = repmat(u0, 1, length(tgrid));

% Solver options settings
problem.options.nlpOpt = optimset(...
    'Display', 'iter', ...
    'MaxFunEvals', 1e3, ...
    'PlotFcns','optimplotfval');
problem.options.method = 'rungeKutta';
problem.options.rungeKutta.nSegment = 10;

% problem.options.method = 'trapezoid';
% problem.options.trapezoid.nGrid = round(N/2);

% Solve
soln = optimTraj(problem);


%% Visualization
t = linspace(soln.grid.time(1), soln.grid.time(end), N);
z = soln.interp.state(t);
u = soln.interp.control(t);

%% Animation
% Get reference
% State
Q1 = xref(1, :);
Q2 = xref(2, :);
Q3 = xref(3, :);
Q4 = xref(4, :);
Q5 = xref(5, :);
V1 = xref(6, :);
V2 = xref(7, :);
V3 = xref(8, :);
V4 = xref(9, :);
V5 = xref(10, :);

% Points of interest
P1x = L1*cos(Q3) + Q1;
P1y = L1*sin(Q3) + Q2;
P2x = Q1;
P2y = Q2;
P3x = -L2*cos(Q3 + Q4) + Q1;
P3y = -L2*sin(Q3 + Q4) + Q2;
P4x = -L2*cos(Q3 + Q4) - L3*cos(Q3 + Q4 + Q5) + Q1;
P4y = -L2*sin(Q3 + Q4) - L3*sin(Q3 + Q4 + Q5) + Q2;
% Ao_x = r1*cos(Q3) + Q1;
% Ao_y = r1*sin(Q3) + Q2;
% Bo_x = (-L2 + r2)*cos(Q3 + Q4) + Q1;
% Bo_y = (-L2 + r2)*sin(Q3 + Q4) + Q2;
% Co_x = -L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5) + Q1;
% Co_y = -L2*sin(Q3 + Q4) + (-L3 + r3)*sin(Q3 + Q4 + Q5) + Q2;

% Get results
% State
Q1_track = z(1, :);
Q2_track = z(2, :);
Q3_track = z(3, :);
Q4_track = z(4, :);
Q5_track = z(5, :);
V1_track = z(6, :);
V2_track = z(7, :);
V3_track = z(8, :);
V4_track = z(9, :);
V5_track = z(10, :);

% Points of interest
P1x_track = L1*cos(Q3_track) + Q1_track;
P1y_track = L1*sin(Q3_track) + Q2_track;
P2x_track = Q1_track;
P2y_track = Q2_track;
P3x_track = -L2*cos(Q3_track + Q4_track) + Q1_track;
P3y_track = -L2*sin(Q3_track + Q4_track) + Q2_track;
P4x_track = -L2*cos(Q3_track + Q4_track) - L3*cos(Q3_track + Q4_track + Q5_track) + Q1_track;
P4y_track = -L2*sin(Q3_track + Q4_track) - L3*sin(Q3_track + Q4_track + Q5_track) + Q2_track;

Pc1x_track = -L2.*cos(Q3_track + Q4_track) - L3.*cos(Q3_track + Q4_track + Q5_track) + Q1_track;
Pc1y_track = -L2.*sin(Q3_track + Q4_track) - L3.*sin(Q3_track + Q4_track + Q5_track) + Q2_track;
Pc2x_track = Q1_track;
Pc2y_track = Q2_track;
Vc1x_track = L2.*(V3_track + V4_track).*sin(Q3_track + Q4_track) + L3.*(V3_track + V4_track + V5_track).*sin(Q3_track + Q4_track + Q5_track) + V1_track;
Vc1y_track = -L2.*(V3_track + V4_track).*cos(Q3_track + Q4_track) - L3.*(V3_track + V4_track + V5_track).*cos(Q3_track + Q4_track + Q5_track) + V2_track;
Vc2x_track = V1_track;
Vc2y_track = V2_track;

% Get contact forces
GRFx = zeros(size(P4x_track));
GRFy = zeros(size(P4y_track));
SFx = zeros(size(P2x_track));
SFy = zeros(size(P2y_track));
for i = 1:N
    if Pc1y_track(i) > ground_height
        GRFy(i) = 0;
        GRFx(i) = 0;
    else
        GRFy(i) = (K_foot*(Pc1y_track(i) - ground_height)^2)*exp(-b_foot*Vc1y_track(i));
        GRFx(i) = -mu_foot*GRFy(i)*tanh(Vc1x_track(i)/0.01);
        % disp('touch the foot')
    end

    % Hip contact model
    if Pc2y_track(i) > seat_height
        SFy(i) = 0;
        SFx(i) = 0;
        % disp('no touch')
    else
        SFy(i) = (K_hip*(Pc2y_track(i) - seat_height)^2)*exp(-b_hip*Vc2y_track(i));
        SFx(i) = -mu_hip*SFy(i)*tanh(Vc2x_track(i)/0.01);
        % disp('touch the butt')
    end
end

% Start anime
animeVideo = VideoWriter(video_name, 'MPEG-4');
animeVideo.FrameRate = 30;
open(animeVideo);

grayColor = [.7 .7 .7];
copperColor = [184, 115, 51]./255;

figure('Position', [10 10 1400 900])
for i = 1:N
    hold on
    xlim([-1.5, 1.5])
    ylim([-1.0, 2])
    axis equal
    
    plot([-100 100], [ground_height ground_height], 'Color', 'g', 'LineWidth', 10) % ground
    plot([-100 -0.2], [seat_height seat_height], 'Color', copperColor, 'LineWidth', 7) % chair
    plot([-0.2 -0.2], [ground_height seat_height], 'Color', copperColor, 'LineWidth', 7) % chair

    % OpenPose
    plot([P1x(i) P2x(i)], [P1y(i) P2y(i)], 'r', 'LineWidth', 10) % trunk
    plot([P2x(i) P3x(i)], [P2y(i) P3y(i)], 'r', 'LineWidth', 10) % thigh
    plot([P3x(i) P4x(i)], [P3y(i) P4y(i)], 'r', 'LineWidth', 10) % shank
    scatter([P2x(i) P3x(i)], [P2y(i) P3y(i)], 300, ...
        'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % hip and knee
    scatter([P4x(i)], [P4y(i)], 300, 'square', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % foot
    scatter([P1x(i)], [P1y(i)], 900, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % head
    
    % Dynamic model
    plot([P1x_track(i) P2x_track(i)], [P1y_track(i) P2y_track(i)], 'b', 'LineWidth', 10) % trunk
    plot([P2x_track(i) P3x_track(i)], [P2y_track(i) P3y_track(i)], 'b', 'LineWidth', 10) % thigh
    plot([P3x_track(i) P4x_track(i)], [P3y_track(i) P4y_track(i)], 'b', 'LineWidth', 10) % shank
    scatter([P2x_track(i) P3x_track(i)], [P2y_track(i) P3y_track(i)], 300, ...
        'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % hip and knee
    scatter([P4x_track(i)], [P4y_track(i)], 300, 'square', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % foot
    scatter([P1x_track(i)], [P1y_track(i)], 900, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % head
    quiver(P4x_track(i), P4y_track(i), GRFx(i)/(1*m*g), GRFy(i)/(1*m*g), ...
        '-', 'LineWidth', 7, 'Color', grayColor)
    quiver(P2x(i), P2y(i), SFx(i)/(1*m*g), SFy(i)/(1*m*g), ...
        '-', 'LineWidth', 7, 'Color', grayColor)

    % hold off
    % alpha(.5)

    pause(0.01);

    if i < N
        frame = getframe(gcf);
        writeVideo(animeVideo, frame);
        clf('reset')
    end

end

close(animeVideo);


%% Post visualization
% Read mocap data (ground truth)
xtruth = readmatrix('data_truth.csv');
xtruth = xtruth(tracking_start:tracking_stop, :);
xtruth = xtruth(:, [12, 13]);
xtruth = xtruth';

knee_truth = -1*xtruth(1, :);
hip_truth = xtruth(2, :);

disp('RMSE of hip angle')
rmse(rad2deg(Q4), rad2deg(hip_truth))
rmse(rad2deg(Q4_track), rad2deg(hip_truth))
disp('RMSE of knee angle')
rmse(rad2deg(Q5), rad2deg(knee_truth))
rmse(rad2deg(Q5_track), rad2deg(knee_truth))


% Joint angles
figure()
subplot(2, 1, 1)
hold on
plot(rad2deg(Q4), 'Color', 'r', 'LineWidth', 1.5)
plot(rad2deg(Q4_track), 'Color', 'b', 'LineWidth', 1.5)
plot(rad2deg(hip_truth), 'Color', grayColor, 'LineWidth', 1.5)
xlim([1, N])
% ylabel('Hip angle (degrees)')
% legend('OpenPose', 'Dynamic model', 'Ground truth')

subplot(2, 1, 2)
hold on
plot(rad2deg(Q5), 'Color', 'r', 'LineWidth', 1.5)
plot(rad2deg(Q5_track), 'Color', 'b', 'LineWidth', 1.5)
plot(rad2deg(knee_truth), 'Color', grayColor, 'LineWidth', 1.5)
% ylabel('Knee angle (degrees)')
xlim([1, N])
% legend('OpenPose', 'Dynamics refinement')

% Joint torques
figure()
subplot(2, 1, 1)
plot(u(5, :), 'LineWidth', 1.5, 'Color', 'b')
xlim([1, N])
set(gca, 'box', 'off')
% ylabel('Hip torque (N.m/kg)')

subplot(2, 1, 2)
plot(u(6, :), 'LineWidth', 2, 'Color', 'b')
xlim([1, N])
% ylabel('Knee torque (N.m/kg)')
set(gca, 'box', 'off')

% GRF and SF
figure()
subplot(2, 1, 1)
hold on
plot(GRFx, '--', 'LineWidth', 1.5, 'Color', 'b')
plot(GRFy, 'LineWidth', 1.5, 'Color', 'b')
xlim([0, N])
% ylabel('GRF (N)')
% legend('Horizontal', 'Vertical')

subplot(2, 1, 2)
hold on
plot(SFx, '--', 'LineWidth', 1.5, 'Color', 'b')
plot(SFy, 'LineWidth', 1.5, 'Color', 'b')
xlim([0, N])
% ylabel('SF (N)')
% legend('Horizontal', 'Vertical')

%% Export tracking results
fn_openpose = 'tracking_op.csv';
fn_tracking_state = 'tracking_model_state.csv';
fn_tracking_control = 'tracking_mocel_control.csv';

writematrix(xref, fn_openpose);
writematrix(z, fn_tracking_state);
writematrix(u, fn_tracking_control);

%% Functions
% Objective (or cost) function
% x (10xN)
function J = quadratic_cost(x, u, xref, xpos)

    N = size(x, 2);
    num_state = size(x, 1);
    
    % Tracking cost
    xref_scaled = zeros(size(x));    
    for i = 1:num_state
        xref_scaled(i, :) = resample(xref(i, :), N, size(xref, 2));        
    end  
    

    J_tracking = (x - xref_scaled).^2;
    J_tracking = sum(J_tracking);

    % Effort cost
    J_effort = (0.01.*u).^2; % bring u the similar scale with position
    J_effort = sum(J_effort);

    J = J_tracking + J_effort;
    % J = J_effort; % TEST WITH J_EFFORT ONLY
    % J = J_tracking; % TEST WITH J_TRACKING ONLY
    
    % % additionally track position
    % model_params;
    % 
    % % State
    % Q1 = x(1, :);
    % Q2 = x(2, :);
    % Q3 = x(3, :);
    % Q4 = x(4, :);
    % Q5 = x(5, :);
    % 
    % % Points of interest
    % P1x = L1.*cos(Q3) + Q1;
    % P1y = L1.*sin(Q3) + Q2;
    % P3x = -L2.*cos(Q3 + Q4) + Q1;
    % P3y = -L2.*sin(Q3 + Q4) + Q2;
    % P4x = -L2.*cos(Q3 + Q4) - L3.*cos(Q3 + Q4 + Q5) + Q1;
    % P4y = -L2.*sin(Q3 + Q4) - L3.*sin(Q3 + Q4 + Q5) + Q2;
    % add_pos = [P1x; P1y; P3x; P3y; P4x; P4y];
    % 
    % xpos_scaled = zeros(size(xpos));
    % for i = 1:size(xpos, 1)
    %     xpos_scaled(i, :) = resample(xpos(i, :), N, size(xpos, 2));
    % end
    % 
    % J_pos = (add_pos - xpos_scaled).^2;
    % J_pos = sum(J_pos);
    % 
    % J = J_tracking + 0.1.*J_effort + J_pos;
end

% Equality constraints (or boundary constraints)

% Control bounds 

% Collocation contraints








