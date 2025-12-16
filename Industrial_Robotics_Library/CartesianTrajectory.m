function traj = CartesianTrajectory(Xstart, Xend, Tf, N, method)
% *** CHAPTER 9: TRAJECTORY GENERATION ***
% Takes Xstart: The initial end-effector configuration,
%       Xend: The final end-effector configuration,
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
% Returns traj: The discretized trajectory as a list of N matrices in SE(3)
%               separated in time by Tf/(N-1). The first in the list is 
%               Xstart and the Nth is Xend .
% This function is similar to ScrewTrajectory, except the origin of the 
% end-effector frame follows a straight line, decoupled from the rotational
% motion.
% Example Input:
% 
% clear; clc;
% Xstart = [[1, 0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 1]; [0, 0, 0, 1]];
% Xend = [[0, 0, 1, 0.1]; [1, 0, 0, 0]; [0, 1, 0, 4.1]; [0, 0, 0, 1]];
% Tf = 5;
% N = 4;
% method = 5;
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method)
% 
% Output:
% traj =
%    1.0000         0         0    1.0000
%         0    1.0000         0         0
%         0         0    1.0000    1.0000
%         0         0         0    1.0000
%
%    0.9366   -0.2140    0.2774    0.8111
%    0.2774    0.9366   -0.2140         0
%   -0.2140    0.2774    0.9366    1.6506
%         0         0         0    1.0000
%
%    0.2774   -0.2140    0.9366    0.2889
%    0.9366    0.2774   -0.2140         0
%   -0.2140    0.9366    0.2774    3.4494
%         0         0         0    1.0000
%
%   -0.0000    0.0000    1.0000    0.1000
%    1.0000   -0.0000    0.0000         0
%    0.0000    1.0000   -0.0000    4.1000
%         0         0         0    1.0000

timegap = Tf / (N - 1);
traj = cell(1, N);
[Rstart, pstart] = TransToRp(Xstart);
[Rend, pend] = TransToRp(Xend);
for i = 1: N
    if method == 3
        s = CubicTimeScaling(Tf,timegap * (i - 1));
    else
        s = QuinticTimeScaling(Tf,timegap * (i - 1));
    end
    traj{i} ...
    = [Rstart * MatrixExp3(MatrixLog3(Rstart' * Rend) * s), ...
       pstart + s * (pend - pstart); 0, 0, 0, 1];
end
end

function [s, sdot] = CubicTimeScaling(Tf, t)
    % Inputs: Tf = Total time, t = Current time
    % Outputs: s = Path parameter, sdot = Time derivative
    
    % Clamp t to [0, Tf]
    t = max(0, min(Tf, t));
    
    % Peak velocity v_max occurs at Tf/2
    % Area under velocity triangle must be 1.
    % 0.5 * base * height = 1 => 0.5 * Tf * v_max = 1 => v_max = 2/Tf
    v_max = 2 / Tf;
    
    if t <= Tf / 2
        % Acceleration Phase
        % v(t) = m*t, where m = v_max / (Tf/2) = 4/Tf^2
        sdot = (4 / Tf^2) * t;
        s = (2 / Tf^2) * t^2;
    else
        % Deceleration Phase
        % Symmetric to acceleration
        sdot = v_max - (4 / Tf^2) * (t - Tf/2);
        % s(t) = 1 - (distance remaining)
        time_left = Tf - t;
        s = 1 - (2 / Tf^2) * time_left^2;
    end
end