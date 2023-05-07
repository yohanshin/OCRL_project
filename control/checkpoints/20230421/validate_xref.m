% name: validate_xref.m
% description: Visualize and validate reference trajectory from OpenPose
% author: Vu Phan
% date: 2023/04/19


model_params; % Initialize model parameters

%% Get tracking data
xref = readmatrix('data.csv');
% xref = xref(300:1340, :);
xref = xref';

N = size(xref, 2); % number of samples

% State
Q1 = xref(1, :);
Q2 = xref(2, :);
Q3 = xref(3, :);
Q4 = xref(4, :);
Q5 = xref(5, :) - pi;
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

%% Animation
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



