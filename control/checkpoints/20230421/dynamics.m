% name: sts_5dof_dyn.m
% description: Dynamics of the 5-DOF sit-to-stand model
% author: Vu Phan
% date: 2023/04/15

% References
% TBD

function xd = dynamics(x, u)   

    model_params; % Initialize model parameters

    % State
    Q1 = x(1);
    Q2 = x(2);
    Q3 = x(3);
    Q4 = x(4);
    Q5 = x(5);
    V1 = x(6);
    V2 = x(7);
    V3 = x(8);
    V4 = x(9);
    V5 = x(10);
    
    % Applied forces
    Fx1 = u(1);
    Fy1 = u(2);
    Fx2 = u(3);
    Fy2 = u(4);
    tau1 = u(5);
    tau2 = u(6);
    
    % Mass matrix
    M(1, 1) = m1 + m2 + m3;
    M(1, 2) = 0;
    M(1, 3) = -m1*r1*sin(Q3) - m2*(-L2 + r2)*sin(Q3 + Q4) + m3*(L2*sin(Q3 + Q4) + (L3 - r3)*sin(Q3 + Q4 + Q5));
    M(1, 4) = -m2*(-L2 + r2)*sin(Q3 + Q4) + m3*(L2*sin(Q3 + Q4) + (L3 - r3)*sin(Q3 + Q4 + Q5));
    M(1, 5) = -m3*(-L3 + r3)*sin(Q3 + Q4 + Q5);
    M(2, 1) = 0;
    M(2, 2) = m1 + m2 + m3;
    M(2, 3) = m1*r1*cos(Q3) + m2*(-L2 + r2)*cos(Q3 + Q4) + m3*(-L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5));
    M(2, 4) = m2*(-L2 + r2)*cos(Q3 + Q4) + m3*(-L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5));
    M(2, 5) = m3*(-L3 + r3)*cos(Q3 + Q4 + Q5);
    M(3, 1) = -m1*r1*sin(Q3) - m2*(-L2 + r2)*sin(Q3 + Q4) + m3*(L2*sin(Q3 + Q4) + (L3 - r3)*sin(Q3 + Q4 + Q5));
    M(3, 2) = m1*r1*cos(Q3) + m2*(-L2 + r2)*cos(Q3 + Q4) + m3*(-L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5));
    M(3, 3) = I1 + I2 + I3 + m1*r1^2 + m2*(-L2 + r2)^2 + m3*(L2^2 - 2*L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(3, 4) = I2 + I3 + m2*(-L2 + r2)^2 + m3*(L2^2 - 2*L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(3, 5) = I3 + m3*(-L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(4, 1) = -m2*(-L2 + r2)*sin(Q3 + Q4) + m3*(L2*sin(Q3 + Q4) + (L3 - r3)*sin(Q3 + Q4 + Q5));
    M(4, 2) = m2*(-L2 + r2)*cos(Q3 + Q4) + m3*(-L2*cos(Q3 + Q4) + (-L3 + r3)*cos(Q3 + Q4 + Q5));
    M(4, 3) = I2 + I3 + m2*(-L2 + r2)^2 + m3*(L2^2 - 2*L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(4, 4) = I2 + I3 + m2*(-L2 + r2)^2 + m3*(L2^2 - 2*L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(4, 5) = I3 + m3*(-L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(5, 1) = -m3*(-L3 + r3)*sin(Q3 + Q4 + Q5);
    M(5, 2) = m3*(-L3 + r3)*cos(Q3 + Q4 + Q5);
    M(5, 3) = I3 + m3*(-L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(5, 4) = I3 + m3*(-L2*(-L3 + r3)*cos(Q5) + (-L3 + r3)^2);
    M(5, 5) = I3 + m3*(-L3 + r3)^2;
    
    
    % Force matrix
    F(1) = m1*r1*V3^2*cos(Q3) + m2*(-L2 + r2)*(V3 + V4)^2*cos(Q3 + Q4) + m3*(-L3 + r3)*(V3 + V4 + V5)^2*cos(Q3 + Q4 + Q5) + m3*(-r2*(V3 + V4) + (-L2 + r2)*(V3 + V4))*(V3 + V4)*cos(Q3 + Q4) + Fx1 + Fx2;
    F(2) = -g*m1 - g*m2 - g*m3 + m1*r1*V3^2*sin(Q3) + m2*(-L2 + r2)*(V3 + V4)^2*sin(Q3 + Q4) + m3*(-L3 + r3)*(V3 + V4 + V5)^2*sin(Q3 + Q4 + Q5) + m3*(-r2*(V3 + V4) + (-L2 + r2)*(V3 + V4))*(V3 + V4)*sin(Q3 + Q4) + Fy1 + Fy2;
    F(3) = L2*g*m3*cos(Q3 + Q4) - L2*m3*(-L3 + r3)*(V3 + V4 + V5)^2*sin(Q5) + L2*Fx1*sin(Q3 + Q4) - L2*Fy1*cos(Q3 + Q4) + L3*Fx1*sin(Q3 + Q4 + Q5) - L3*Fy1*cos(Q3 + Q4 + Q5) - g*m1*r1*cos(Q3) - g*m2*(-L2 + r2)*cos(Q3 + Q4) - g*m3*(-L3 + r3)*cos(Q3 + Q4 + Q5) - m3*(-L3 + r3)*(-r2*(V3 + V4) + (-L2 + r2)*(V3 + V4))*(V3 + V4)*sin(Q5) + 2*tau2;
    F(4) = L2*g*m3*cos(Q3 + Q4) - L2*m3*(-L3 + r3)*(V3 + V4 + V5)^2*sin(Q5) + L2*Fx1*sin(Q3 + Q4) - L2*Fy1*cos(Q3 + Q4) + L3*Fx1*sin(Q3 + Q4 + Q5) - L3*Fy1*cos(Q3 + Q4 + Q5) - g*m2*(-L2 + r2)*cos(Q3 + Q4) - g*m3*(-L3 + r3)*cos(Q3 + Q4 + Q5) - m3*(-L3 + r3)*(-r2*(V3 + V4) + (-L2 + r2)*(V3 + V4))*(V3 + V4)*sin(Q5) - tau1 + 2*tau2;
    F(5) = L3*Fx1*sin(Q3 + Q4 + Q5) - L3*Fy1*cos(Q3 + Q4 + Q5) - g*m3*(-L3 + r3)*cos(Q3 + Q4 + Q5) - m3*(-L3 + r3)*(-r2*(V3 + V4) + (-L2 + r2)*(V3 + V4))*(V3 + V4)*sin(Q5) + tau2;
%     F = F;

    % Calculate generalized accelerations
    Vd = M\F';

    % Calculate generalized speeds
    V = [V1; V2; V3; V4; V5];
    
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
    
    % Velocities of interest
    Pc1x = -L2*cos(Q3 + Q4) - L3*cos(Q3 + Q4 + Q5) + Q1;
    Pc1y = -L2*sin(Q3 + Q4) - L3*sin(Q3 + Q4 + Q5) + Q2;
    Pc2x = Q1;
    Pc2y = Q2;
    Vc1x = L2*(V3 + V4)*sin(Q3 + Q4) + L3*(V3 + V4 + V5)*sin(Q3 + Q4 + Q5) + V1;
    Vc1y = -L2*(V3 + V4)*cos(Q3 + Q4) - L3*(V3 + V4 + V5)*cos(Q3 + Q4 + Q5) + V2;
    Vc2x = V1;
    Vc2y = V2;

    % Output rate
    xd = [V; Vd];

end





