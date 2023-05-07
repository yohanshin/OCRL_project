% name: dynamics_n.m
% description: Compute dynamics for all knot points
% author: Vu Phan
% date: 2023/04/18


function xd = dynamics_n(x, u)   
    N = length(x); % number of knot points
    
    xd = zeros(size(x));

    for k = 1:N
        xd(:, k) = dynamics(x(:, k), u(:, k));
    end

    
end


