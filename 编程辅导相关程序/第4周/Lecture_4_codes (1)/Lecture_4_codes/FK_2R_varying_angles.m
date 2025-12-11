% computes the forward kinematics of a planar 2R manipulator.
% It assigns arbitrary frames to the link/joints and
% computes the rotation and homogeneous transformation matrices.
% This code is meant to show how we can perform the FK simply by knowledge
% of the homogeneous transformation matrices in a simple planar (2D) case
% and the proceed to plot the workspace of the manipulator
% This script defines varying joint angles at different interval in order
% to show the motion of the manipulator
clear all
close all
volume_length = 1.;
% initial geometrical parameter
L1 = 0.5;       % link 1
L2 = 0.4;       % link 2
theta1 = deg2rad(30);   % initial angle of joint 1
theta2 = deg2rad(20);   % initial angle of joint 2

% set up figures
figure(1)
plotvol(volume_length);
x0=100;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
hold on
view(0,90)
trplot(eye(4), 'length', L2/2)
% ----------------------------
figure(2)
plotvol(volume_length);
view(0,90)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
trplot(eye(4), 'length', L2/2)



for i = 1:20
    % below we specify how we want the joint angles to change in each
    % interval:
    %theta1 = theta1+deg2rad(5);  % this adds 5deg to joint 1 each time
    %theta2 = theta2+deg2rad(10); % this adds 10deg to joint 2 each time
    %---------------------------------------------------------------------
    % an alternative way of assigning angles to the joint could be to
    % assign a random angle between 0 and 360degrees
    theta1 = deg2rad(rand*360);
    theta2 = deg2rad(rand*360);
    %---------------------------------------------------------------------
    % an alternative way of assigning angles to the joint could be to
    % assign a harmonic function so that the angles vary periodically
    %theta1 = cos(0.3*i);
    %theta2 = cos(0.5*i);
    %---------------------------------------------------------------------
    % calulation of rotation matrices, displacement vectors and HTMs
    R01 = rotz(theta1);
    d01 = [L1*cos(theta1); L1*sin(theta1); 0];
    R12 = rotz(theta2);
    d12 = [L2*cos(theta2); L2*sin(theta2); 0];
    T01 = RpToTrans(R01, d01);
    T12 = RpToTrans(R12, d12);
    T02 = T01*T12;

    figure(1)
    plotvol(volume_length);
    hold on
    view(0,90)
    trplot(eye(4), 'length', L2/2)
    trplot(T01, 'color','r', 'length', volume_length/3)
    trplot(T02,'color','k', 'length', L2/2)
    plot3([0 T01(1,4)], [0 T01(2,4)], [0 T01(3,4)], 'k')
    plot3([T01(1,4) T02(1,4)], [T01(2,4) T02(2,4)], [T01(3,4) T02(3,4)],'k')
    plot_sphere([0  0 0], L2/15)
    plot_sphere([T01(1,4)  T01(2,4) T01(3,4)], L2/15)
    plot_sphere([T02(1,4)  T02(2,4) T02(3,4)], L2/15)
    drawnow
    %pause(0.05)
    hold off

    figure(2)
    plot_sphere([T02(1,4)  T02(2,4) T02(3,4)], L2/20, 'r')
    hold on
    trplot(T02,'color','k', 'length', L2/5)
    pause(0.05)
end