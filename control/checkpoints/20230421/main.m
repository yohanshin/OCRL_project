% name: main.m
% description: Perform tracking with LQR
% author: Vu Phan
% date: 2023/04/15


clear all
close all
clc


model_params; % Initialize model parameters

% Simulation settings
t_sim = 0:0.001:1.0;

% Initial state
Q1 = 0.0;
Q2 = 0.8;
Q3 = 90.0*pi/180.0;
Q4 = 40.0*pi/180.0;
Q5 = -60.0*pi/180.0;
V1 = 0;
V2 = 0;
V3 = 0;
V4 = 0;
V5 = 0;
x0 = [Q1, Q2, Q3, Q4, Q5, V1, V2, V3, V4, V5];

hold on
xlim([-1.5, 1.5])
ylim([-0.5, 2])
axis equal

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

plot([-100 100], [0 0], 'g', 'LineWidth', 2) % ground
plot([P1x P2x], [P1y P2y], 'b', 'LineWidth', 2) % trunk
plot([P2x P3x], [P2y P3y], 'b', 'LineWidth', 2) % trunk
plot([P3x P4x], [P3y P4y], 'b', 'LineWidth', 2) % trunk
scatter([P1x P2x P3x P4x], [P1y P2y P3y P4y], ...
        'MarkerFaceColor', 'k') % joints

% % Option
% options = odeset('AbsTol', 1e-08, 'RelTol', 1e-08);
% [t, x] = ode45(@sts_5dof_dyn, t_sim, x0, options);
% 
% % State
% Q1 = x(:, 1);
% Q2 = x(:, 2);
% Q3 = x(:, 3);
% Q4 = x(:, 4);
% Q5 = x(:, 5);
% V1 = x(:, 6);
% V2 = x(:, 7);
% V3 = x(:, 8);
% V4 = x(:, 9);
% V5 = x(:, 10);
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
% % Video
% animeVideo = VideoWriter('animeVideo', 'MPEG-4');
% animeVideo.FrameRate = 24;
% open(animeVideo);
% 
% figure()
% for i = 1:10:length(t_sim)
%     hold on
%     xlim([-1.5, 1.5])
%     ylim([-0.5, 2])
%     axis equal
% 
%     plot([-100 100], [0 0], 'g', 'LineWidth', 2) % ground
%     plot([P1x(i) P2x(i)], [P1y(i) P2y(i)], 'b', 'LineWidth', 2) % trunk
%     plot([P2x(i) P3x(i)], [P2y(i) P3y(i)], 'b', 'LineWidth', 2) % trunk
%     plot([P3x(i) P4x(i)], [P3y(i) P4y(i)], 'b', 'LineWidth', 2) % trunk
% 
%     pause(0.01);
% 
%     if i < length(t)
%         frame = getframe(gcf);
%         writeVideo(animeVideo, frame);
%         clf('reset')
%     end
% 
% end
% 
% close(animeVideo);






