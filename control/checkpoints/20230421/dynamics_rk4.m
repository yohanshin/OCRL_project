% name: dynamics_rk4.m
% description: Descrete dynamics of the 5-DOF sit-to-stand model
% author: Vu Phan
% date: 2023/04/15

function xd = dynamics_rk4(x, u, dt)

    model_params;
    
    % Vanilla RK4
    k1 = dt*dynamics(x, u);
    k2 = dt*dynamics(x + k1/2, u);
    k3 = dt*dynamics(x + k2/2, u);
    k4 = dt*dynamics(x + k3, u);
    
    xd = x + (k1 + 2*k2 + 2*k3 + k4)/6;
end




