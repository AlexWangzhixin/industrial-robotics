% this code shows how to model the angular position of a joint given the
% angular velocity of that joint
clear all
close all

theta0 = 0; % [rad] initial angular position of the joint
thetadot = deg2rad(40) % [rad/sec] angular velocity of the joint

Tin = 0.;                   % initial time of the simulation
Tfin = 30.0;                 % final time of the simulation
DeltaT = 0.1;               % time increment
N = floor((Tfin-Tin)/DeltaT) % number of timesteps

theta_temp = theta0;        % initialize a temporary theta value based on the initial theta
theta = zeros(N);           % creates array where to store the values of angular position of the joint
time  = zeros(N)            % creates a time array where to store the timestamp

for i=1:N
    time(i) = i*DeltaT;
    % example of joint angle variation with a linear velocity
    theta_temp = theta_temp + thetadot*DeltaT; % compute the theta increment due to the angular velocity
    theta(i) = wrapTo2Pi(theta_temp); % stores the temporary values of theta 
    % example of joint angle variation with a harmonic velocity
    %theta_temp = sin(thetadot*DeltaT*i);    
    %theta(i) = theta_temp;
end

figure(1)
plot(time, rad2deg(theta))
xlabel('time [s]')
ylabel('angular position [deg]')