% computes the forward velocity kinematics of a planar 2R manipulator.
% Here we use the analytical expression of the Jacobian which we derived in
% class.
% The FK is achieved by knowledge
% of the homogeneous transformation matrices. We can do the calculation of
% the jacobian in this case because the
% manipulator is a simple planar case.
clear all
close all
format short

% assign the geometrical parameters of the manipulator. In this case they
% are only the distances between joint 1 and joint 2:
L1 = 0.5;
L2 = 0.4;
% assigns the rotation of two joint angles
theta1 = deg2rad(40);
theta2 = deg2rad(60);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          Compute forward kinematics given L1, L2, theta1, theta2
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% compute the rotation matrices and displacement vectors of the two joints
% and the end-effector, which in this case is aligned exactly like joint-2
% and displaced L2 from it.
Rs0 = rotz(theta1); % rotation of joint-1 wrt base
ds0 = [0;0;0];      % displacement of joint-1 wrt base
R01 = rotz(theta2); % rotation of joint-2 wrt base
d01 = [L1;0; 0];    % displacement of joint-2 wrt base
R12 = eye(3); % <- this command simply says that the end-effector has the same orientation of the previous joint
d12 = [L2; 0; 0]; % <- the end-effector is displaced L2 from joint-2 by means of the third link of the manipulator

% computes the HTM for joint-1, joint-2 and the end-effector
Ts0 = RpToTrans(Rs0,ds0);
T01 = RpToTrans(R01, d01);
T12 = RpToTrans(R12, d12); 
Ts1 = Ts0*T01; % transformation from the base to joint-1
Ts2 = Ts1*T12; % this is the solution to the FK problem for the end-effector: from base to end-effector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          Compute Jacobian and forward velocity kinematics given dot_theta1, dot_theta2
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% here starts the part of the code which computes the Forward Velocity
% Kinematics
% Fist we compute the vector form of the end-effector by extracting it from Ts2
x_e = zeros(6,1)                        % this is the array that contains the x, y, z, alpha, beta, gamma of the end-effector
x_e(1:3) = Ts2(1:3,4)                   % the position part of the vector can be extracted directly from the HTM
[Rs2,ds2]=TransToRp(Ts2)                % for the rotational part we need to first extract the rotation matrix of Ts2
euler_angles = rotm2eul(Rs2, 'ZYX');    % then from the rotation matrix of Ts2 we apply the inverse of the Euler angles XYZ
x_e(4)= euler_angles(3)                 % this command takes the third element of euler_angles, the rotation around x
x_e(5)= euler_angles(2)                 % this takes the rotation around y
x_e(6)= euler_angles(1)                 % this takes the rotation around z
% now that we have the vector describing the end-effector position and
% orientations (expressed in XYZ Euler angles) we can
% compute the jacobian of dot_x_e. We know the Jacobian because we derived
% analytically in class, we could derive it analytically because the
% geometry of this manipulator in simple
J = [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2);...
     L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2)]
% we assign some arbitrary angular velocity to the joint-1 and joint-2 to
% test the code:
dot_theta1 = 3.14/10; % rad/sec
dot_theta2 = 6.28/10; % rad/sec
% put the two joint velocities in a single vector dot_theta
dot_theta = [dot_theta1;dot_theta2];
% then we can solve the forward velocity kinematics:
dot_x_e = J*dot_theta


% we now perform some plotting to visualize the results
volume_length = 1.5; % this command specifies the size of the plotting domain
figure(1) % in this figure we plot the two components of the Jacobian separately
x0=200;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(volume_length);
hold on
view(0,90)
trplot(eye(4),'frame', 's', 'length', L2/2)
trplot(Ts1, 'frame', '1','color','r', 'length', L2/2)
trplot(Ts2,'frame', '2','color','r','length', L2/2)
plot3([0 Ts1(1,4)], [0 Ts1(2,4)], [0 Ts1(3,4)], 'k')
plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k')
plot_sphere([0  0 0], L2/15)
plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
% plot the Jacobian components
plot_arrow(x_e(1:2),x_e(1:2)+J(1:2,1),'r', 0.4)% first column of the Jacobian, represents velocity of the end-effector if velocity of joint-2 is zero
plot_arrow(x_e(1:2),x_e(1:2)+J(1:2,2),'b', 0.4)% second column of the Jacobian, represents velocity of the end-effector if velocity of joint-1 is zero
title(strcat({'$J_{1}$ and $J_{2}$ when $\theta_1$ and $\theta_2$ are: '},{num2str(theta1)},'$^\circ$ and ', {num2str(theta2)},'$^\circ$'), 'interpreter', 'latex')
drawnow

volume_length = 1.5;
figure(2) % in this figure we plot the resultant end-effector velocity from the solution of the velocity FK
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(volume_length);
hold on
view(0,90)
trplot(eye(4),'frame', 's', 'length', L2/2)
trplot(Ts1, 'frame', '1','color','r', 'length', L2/2)
trplot(Ts2,'frame', '2','color','r','length', L2/2)
plot3([0 Ts1(1,4)], [0 Ts1(2,4)], [0 Ts1(3,4)], 'k')
plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k')
plot_sphere([0  0 0], L2/15)
plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
% plot the end-effector velocity
plot_arrow(x_e(1:2),x_e(1:2)+dot_x_e(1:2,1),'r', 0.4)% this command lots the velocity vector dot_xe at the location of the end-effector
title(strcat('velocity of the end effector $\dot {\vec x_e}$'),'interpreter','latex')
drawnow
