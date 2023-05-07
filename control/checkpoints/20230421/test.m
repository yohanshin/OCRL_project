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

%% Add OptimTraj toolbox
addpath('OptimTraj-master/OptimTraj-master')  

%% Get tracking data from OpenPose
xref = readmatrix('data.csv');
% xref = xref(300:1340, :);
xref = xref(380:420, :);
xref = xref';

xref(5, :) = xref(5, :) - pi; % BECAUSE OF OUR DATA

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
x_low = [-0.5; 0; 0.0*pi/180.0; -50.0*pi/180.0; -150.0*pi/180.0; -inf; -inf; -inf; -inf; -inf];
x_upp = [0.5; 1; 180.0*pi/180.0; 150*pi/180.0; 50.0*pi/180.0; inf; inf; inf; inf; inf];
% x_low = [-0.5; 0; -inf; -inf; -inf; -inf; -inf; -inf; -inf; -inf];
% x_upp = [0.5; 1; inf; inf; inf; inf; inf; inf; inf; inf];

% Dynamics constraints
u_low = [-700*ones(4, 1); -10*ones(2, 1)];
u_upp = [700*ones(4, 1); 10*ones(2, 1)];

% Others
eps = 1e-6; % perturbation size for forward differences
nx = length(x0); % number of state variables
nu = length(u0); % number of control inputs

%% Direct collocation
% Dynamics and cost settings
problem.func.dynamics = @(t, x, u)(dynamics_n(x, u)); % dynamics
problem.func.pathObj = @(t, x, u)(quadratic_cost(x, u, xref)); % cost function

% Bounds settings 
% problem.bounds.initialTime.low = 0;
% problem.bounds.initialTime.upp = 0;
% problem.bounds.finalTime.low = duration;
% problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = xref(:, 1);
problem.bounds.initialState.upp = xref(:, 1);
problem.bounds.finalState.low = xref(:, end);
problem.bounds.finalState.upp = xref(:, end);

% problem.bounds.state.low = x_low; % x_L <= x <= x_U
% problem.bounds.state.upp = x_upp;
% problem.bounds.control.low = u_low; % u_L <= u <= u_U
% problem.bounds.control.upp = u_upp;

% Initial guess settings
problem.guess.time = tgrid;
% problem.guess.state = repmat(xref(:, 1), 1, length(tgrid));
% problem.guess.control = repmat(u0, 1, length(tgrid));
problem.guess.state = xref;
problem.guess.control = repmat(u0, 1, length(tgrid));

% Solver options settings
problem.options.nlpOpt = optimset(...
    'Display', 'iter', ...
    'MaxFunEvals', 1e5, ...
    'PlotFcns','optimplotfval');
problem.options.method = 'rungeKutta';
% problem.options.rungeKutta.nSegment = round(N/2);
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
Ao_x = r1*cos(Q3) + Q1;
Ao_y = r1*sin(Q3) + Q2;
Bo_x = (-L2 + r2)*cos(Q3 + Q4) + Q1;
Bo_y = (-L2 + r2)*sin(Q3 + Q4) + Q2;
Co_x = -L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5) + Q1;
Co_y = -L2*sin(Q3 + Q4) + (-L3 + r3)*sin(Q3 + Q4 + Q5) + Q2;

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

% Start anime
animeVideo = VideoWriter('tracking_anime_8', 'MPEG-4');
animeVideo.FrameRate = 24;
open(animeVideo);

figure()
for i = 1:N
    hold on
    xlim([-1.5, 1.5])
    ylim([-1.0, 2])
    axis equal
    
    plot([-100 100], [0 0], 'g', 'LineWidth', 2) % ground
    plot([P1x_track(i) P2x_track(i)], [P1y_track(i) P2y_track(i)], 'b', 'LineWidth', 2) % trunk
    plot([P2x_track(i) P3x_track(i)], [P2y_track(i) P3y_track(i)], 'b', 'LineWidth', 2) % thigh
    plot([P3x_track(i) P4x_track(i)], [P3y_track(i) P4y_track(i)], 'b', 'LineWidth', 2) % shank
    scatter([P1x_track(i) P2x_track(i) P3x_track(i) P4x_track(i)], [P1y_track(i) P2y_track(i) P3y_track(i) P4y_track(i)], ...
        'MarkerFaceColor', 'k') % joints

    plot([-100 100], [0 0], 'g', 'LineWidth', 2) % ground
    plot([P1x(i) P2x(i)], [P1y(i) P2y(i)], 'r', 'LineWidth', 2) % trunk
    plot([P2x(i) P3x(i)], [P2y(i) P3y(i)], 'r', 'LineWidth', 2) % thigh
    plot([P3x(i) P4x(i)], [P3y(i) P4y(i)], 'r', 'LineWidth', 2) % shank
    scatter([P1x(i) P2x(i) P3x(i) P4x(i)], [P1y(i) P2y(i) P3y(i) P4y(i)], ...
        'MarkerFaceColor', 'k') % joints

    pause(0.01);

    if i < N
        frame = getframe(gcf);
        writeVideo(animeVideo, frame);
        clf('reset')
    end

end

close(animeVideo);


%% Functions
% Objective (or cost) function
% x (10xN)
function J = quadratic_cost(x, u, xref)

    N = size(x, 2);
    num_state = size(x, 1);
    
    % Tracking cost
    xref_scaled = zeros(size(x));
    for i = 1:num_state
        xref_scaled(1, :) = resample(xref(i, :), N, size(xref, 2));
    end

    J_tracking = (x - xref_scaled).^2;
    J_tracking = sum(J_tracking);

    % Effort cost
    J_effort = u.^2;
    J_effort = sum(J_effort);

    J = J_tracking + J_effort;
    % J = J_effort; % TEST WITH J_EFFORT ONLY
    % J = J_tracking; % TEST WITH J_TRACKING ONLY
end

% Equality constraints (or boundary constraints)

% Control bounds 

% Collocation contraints



% x = zeros(nx, N);
% x(:, 1) = x0;
% 
% Q = diag(ones(nx, 1));
% R = diag(ones(nu, 1));
% Qf = 10*Q;
% 
% P = cell(1, N);
% K = cell(1, N - 1);
% P{N} = Qf;
% 
% disp('*Start LQR')
% for k = (N-1):-1:1
%     A = get_A(xref(:, k), u0, eps, dt);
%     B = get_B(xref(:, k), u0, eps, dt);
%     
%     K{k} = inv(R + B'*P{k+1}*B)*B'*P{k+1}*A;
%     P{k} = Q + A'*P{k+1}*(A - B*K{k});
%     
%     if mod(k, 100) == 0
%         disp(k)
%     end
%     
% end
% disp('-> Finished!')
% 
% % Tracking
% uout = zeros(nu, N - 1); % store all u output
% 
% disp('*Start tracking')
% for k = 1:(N - 1)
%     u = -K{k}*(x(:, k) - xref(:, k));
%     x(:, k + 1) = dynamics_rk4(x(:, k), u, dt);   
%     uout(:, k) = u;
% end
% disp('-> Finished!')

%% Post-processing
% % State
% Q1 = x(1, :);
% Q2 = x(2, :);
% Q3 = x(3, :);
% Q4 = x(4, :);
% Q5 = x(5, :);
% V1 = x(6, :);
% V2 = x(7, :);
% V3 = x(8, :);
% V4 = x(9, :);
% V5 = x(10, :);
% 
% % Points of interest
% P1x = L1*cos(Q3) + Q1;
% P1y = L1*sin(Q3) + Q2;
% P2x = Q1;
% P2y = Q2;
% P3x = -L2*cos(Q3 + Q4) + Q1;
% P3y = -L2*sin(Q3 + Q4) + Q2;
% P4x = -L2*cos(Q3 + Q4) - L3*cos(Q3 + Q4 + Q5) + Q1;
% P4y = -L2*sin(Q3 + Q4) - L3*sin(Q3 + Q4 + Q5) + Q2;
% Ao_x = r1*cos(Q3) + Q1;
% Ao_y = r1*sin(Q3) + Q2;
% Bo_x = (-L2 + r2)*cos(Q3 + Q4) + Q1;
% Bo_y = (-L2 + r2)*sin(Q3 + Q4) + Q2;
% Co_x = -L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5) + Q1;
% Co_y = -L2*sin(Q3 + Q4) + (-L3 + r3)*sin(Q3 + Q4 + Q5) + Q2;
% 
% 
% % Animation
% animeVideo = VideoWriter('animeVideo', 'MPEG-4');
% animeVideo.FrameRate = 24;
% open(animeVideo);
% 
% figure()
% for i = 1:N
%     hold on
%     xlim([-1.5, 1.5])
%     ylim([-60.0, 2])
%     axis equal
%     
%     plot([-100 100], [0 0], 'g', 'LineWidth', 2) % ground
%     plot([P1x(i) P2x(i)], [P1y(i) P2y(i)], 'b', 'LineWidth', 2) % trunk
%     plot([P2x(i) P3x(i)], [P2y(i) P3y(i)], 'b', 'LineWidth', 2) % trunk
%     plot([P3x(i) P4x(i)], [P3y(i) P4y(i)], 'b', 'LineWidth', 2) % trunk
% 
%     pause(0.01);
% 
%     if i < N
%         frame = getframe(gcf);
%         writeVideo(animeVideo, frame);
%         clf('reset')
%     end
% 
% end
% 
% close(animeVideo);






