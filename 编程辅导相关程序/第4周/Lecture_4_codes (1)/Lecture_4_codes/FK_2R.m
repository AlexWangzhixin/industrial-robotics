% computes the forward kinematics of a planar 2R manipulator.
% It assigns arbitrary frames to the link/joints and
% computes the rotation and homogeneous transformation matrices.
% This code is meant to show how we can perform the FK simply by knowledge
% of the homogeneous transformation matrices in a simple planar (2D) case
% and the proceed to plot the workspace of the manipulator
clear all
close all
format short
L1 = 0.5;       % length of link 1
L2 = 0.4;       % length of link 2
% below you can input the desired angle of the two joints theta1 and theta2
theta1 = deg2rad(30);       % desired angle for joint 1
theta2 = deg2rad(-50);       % desired angle for joint 2

% below starts the calculation
Rs0 = rotz(theta1);         % build rotation matrix for base frame
ds0 = [0;0;0];              % displacement vector for base frame
R01 = rotz(theta2);         % build rotation matrix for joint 1
d01 = [L1;0; 0];            % displacement vector for joint 1
R12 = eye(3);               % build rotation matrix for joint 2
d12 = [L2; 0; 0];           % displacement vector for joint 2

Ts0 = RpToTrans(Rs0,ds0);   % build HTM for base frame
T01 = RpToTrans(R01, d01);  % build HTM for joint 1
T12 = RpToTrans(R12, d12);  % build HTM for joint 2
Ts1 = Ts0*T01;              % compute HTM from base to joint 1
Ts2 = Ts1*T12;              % compute HTM from base to joint 1 to joint 2

% below we start the plotting of the manipulator
volume_length = 1.;
plotvol(volume_length);
hold on
view(0,90)
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
trplot(eye(4),'frame', 's', 'length', L2/2)
trplot(Ts1, 'frame', '1','color','r', 'length', L2/2)
trplot(Ts2,'frame', '2','color','r','length', L2/2)
plot3([0 Ts1(1,4)], [0 Ts1(2,4)], [0 Ts1(3,4)], 'k')
plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k')
plot_sphere([0  0 0], L2/15)
plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
drawnow