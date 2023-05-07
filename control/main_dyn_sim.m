% name: main_dyn_sym.m
% description: Simulate dynamics with RK4
% author: Vu Phan
% date: 2023/04/15


clear all
close all
clc


video_fn = 'dyn_update_2_forces';
grayColor = [.7 .7 .7];

%% Init
model_params; % Initialize model parameters

% Simulation settings
dt = 1.0/30.0;
t_sim = 0:dt:1.0;

% Initial state
Q1 = 0.0;
Q2 = 1;
Q3 = 90.0*pi/180.0;
Q4 = 30.0*pi/180.0;
Q5 = -35.0*pi/180.0;
V1 = 0;
V2 = 0;
V3 = 0;
V4 = 0;
V5 = 0;
x0 = [Q1, Q2, Q3, Q4, Q5, V1, V2, V3, V4, V5];

Fx1 = 0;
Fy1 = 0;
Fx2 = 0;
Fy2 = 0;
tau1 = 0;
tau2 = 0;
u = [Fx1, Fy1, Fx2, Fy2, tau1, tau2];

%% Simulation
% Simulate dynamics
N = length(t_sim);
% x = zeros(length(x0), N);
% x(:, 1) = x0;

% for k = 1:(N - 1)
%     % x(:, k + 1) = dynamics_rk4(x(:, k), u, dt);  
% end

[t, x] = ode15s(@(t, x) dynamics(x, u), t_sim, x0); % Use ode15s to avoid singular
x = x';

%% Extract simulation
% State
Q1 = x(1, :);
Q2 = x(2, :);
Q3 = x(3, :);
Q4 = x(4, :);
Q5 = x(5, :);
V1 = x(6, :);
V2 = x(7, :);
V3 = x(8, :);
V4 = x(9, :);
V5 = x(10, :);

% Points of interest
P1x = L1*cos(Q3) + Q1;
P1y = L1*sin(Q3) + Q2;
P2x = Q1;
P2y = Q2;
P3x = -L2*cos(Q3 + Q4) + Q1;
P3y = -L2*sin(Q3 + Q4) + Q2;
P4x = -L2*cos(Q3 + Q4) - L3*cos(Q3 + Q4 + Q5) + Q1;
P4y = -L2*sin(Q3 + Q4) - L3*sin(Q3 + Q4 + Q5) + Q2;

Pc1x = -L2.*cos(Q3 + Q4) - L3.*cos(Q3 + Q4 + Q5) + Q1;
Pc1y = -L2.*sin(Q3 + Q4) - L3.*sin(Q3 + Q4 + Q5) + Q2;
Pc2x = Q1;
Pc2y = Q2;
Vc1x = L2.*(V3 + V4).*sin(Q3 + Q4) + L3.*(V3 + V4 + V5).*sin(Q3 + Q4 + Q5) + V1;
Vc1y = -L2.*(V3 + V4).*cos(Q3 + Q4) - L3.*(V3 + V4 + V5).*cos(Q3 + Q4 + Q5) + V2;
Vc2x = V1;
Vc2y = V2;

% Get contact forces
GRFx = zeros(size(P4x));
GRFy = zeros(size(P4y));
SFx = zeros(size(P2x));
SFy = zeros(size(P2y));
for i = 1:N
    if Pc1y(i) > ground_height
        GRFy(i) = 0;
        GRFx(i) = 0;
    else
        GRFy(i) = (K_foot*(Pc1y(i) - ground_height)^2)*exp(-b_foot*Vc1y(i));
        GRFx(i) = -mu_foot*GRFy(i)*tanh(Vc1x(i)/0.01);
    end

    % Hip contact model
    if Pc2y(i) > seat_height
        SFy(i) = 0;
        SFx(i) = 0;
    else
        SFy(i) = (K_hip*(Pc2y(i) - seat_height)^2)*exp(-b_hip*Vc2y(i));
        SFx(i) = -mu_hip*SFy(i)*tanh(Vc2x(i)/0.01);
    end
end


%% Animation
% Setup
animeVideo = VideoWriter(video_fn, 'MPEG-4');
animeVideo.FrameRate = 24;
open(animeVideo);

% Start
grayColor = [.7 .7 .7];
copperColor = [184, 115, 51]./255;

figure('Position', [10 10 1400 900])
for i = 1:N
    hold on
    xlim([-1.5, 1.5])
    ylim([-1.0, 2])
    axis equal
    
    plot([-100 100], [ground_height ground_height], 'Color', 'g', 'LineWidth', 10) % ground
    plot([-100 0.05], [seat_height seat_height], 'Color', copperColor, 'LineWidth', 7) % chair
    plot([0.05 0.05], [ground_height seat_height], 'Color', copperColor, 'LineWidth', 7) % chair

    % OpenPose
    plot([P1x(i) P2x(i)], [P1y(i) P2y(i)], 'b', 'LineWidth', 10) % trunk
    plot([P2x(i) P3x(i)], [P2y(i) P3y(i)], 'b', 'LineWidth', 10) % thigh
    plot([P3x(i) P4x(i)], [P3y(i) P4y(i)], 'b', 'LineWidth', 10) % shank
    scatter([P2x(i) P3x(i)], [P2y(i) P3y(i)], 300, ...
        'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % hip and knee
    scatter([P4x(i)], [P4y(i)], 300, 'square', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % foot
    scatter([P1x(i)], [P1y(i)], 900, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k') % head

    quiver(P4x(i), P4y(i), GRFx(i)/(1*m*g), GRFy(i)/(1*m*g), ...
        '-', 'LineWidth', 7, 'Color', grayColor)
    quiver(P2x(i), P2y(i), SFx(i)/(1*m*g), SFy(i)/(1*m*g), ...
        '-', 'LineWidth', 7, 'Color', grayColor)

    pause(0.01);

    if i <= N
        frame = getframe(gcf);
        writeVideo(animeVideo, frame);
        clf('reset')
    end

end

close(animeVideo);






