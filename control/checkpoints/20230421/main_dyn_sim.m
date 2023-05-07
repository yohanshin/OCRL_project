% name: main_dyn_sym.m
% description: Simulate dynamics with RK4
% author: Vu Phan
% date: 2023/04/15


clear all
close all
clc


model_params; % Initialize model parameters

% Simulation settings
dt = 1.0/30.0;
t_sim = 0:dt:1.0;

% Initial state
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
x0 = [Q1, Q2, Q3, Q4, Q5, V1, V2, V3, V4, V5];

Fx1 = 0;
Fy1 = 0;
Fx2 = 0;
Fy2 = 0;
tau1 = 0;
tau2 = 0;
u = [Fx1, Fy1, Fx2, Fy2, tau1, tau2];

% Simulate dynamics
N = length(t_sim);
x = zeros(length(x0), N);
x(:, 1) = x0;

for k = 1:(N - 1)
    x(:, k + 1) = dynamics_rk4(x(:, k), u, dt);  
end

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
Ao_x = r1*cos(Q3) + Q1;
Ao_y = r1*sin(Q3) + Q2;
Bo_x = (-L2 + r2)*cos(Q3 + Q4) + Q1;
Bo_y = (-L2 + r2)*sin(Q3 + Q4) + Q2;
Co_x = -L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5) + Q1;
Co_y = -L2*sin(Q3 + Q4) + (-L3 + r3)*sin(Q3 + Q4 + Q5) + Q2;


% Animation
animeVideo = VideoWriter('animeVideo', 'MPEG-4');
animeVideo.FrameRate = 24;
open(animeVideo);

figure()
for i = 1:N
    hold on
    xlim([-1.5, 1.5])
    ylim([-1.0, 2])
    axis equal
    
    plot([-100 100], [0 0], 'g', 'LineWidth', 2) % ground
    plot([P1x(i) P2x(i)], [P1y(i) P2y(i)], 'b', 'LineWidth', 2) % trunk
    plot([P2x(i) P3x(i)], [P2y(i) P3y(i)], 'b', 'LineWidth', 2) % thigh
    plot([P3x(i) P4x(i)], [P3y(i) P4y(i)], 'b', 'LineWidth', 2) % shank
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






