% screw theory IK of a RRPRRR robot

clear all 
close all
format short

% geometrical fixed parameters

L1 = 0.2; % [m]
L2 = 0.1; % [m]
L3 = 0.1; % [m]
L6 = 0.1; % [m]
theta1 = 0; % [rad]
theta2 = 0; % [rad]
theta3 = 0; % [m] because this is a prismatic joint
theta4 = 0; % [rad]
theta5 = 0; % [rad]
theta6 = 0; % [rad]
% define joint positions and orientation at home config
M = RpToTrans(rotx(-pi/2)*rotz(-pi/2), [0 L2+L3+L6 L1]');
M1 = eye(4);
M2 = RpToTrans(eye(3), [0 0 L1]');
M3 = RpToTrans(eye(3), [0 L2 L1]');
% we can define the centre of the wrist altogether
M456 = RpToTrans(eye(3), [0 L2+L3 L1]');
 
% define screw axes at home configuration
% S1 recvolute
omghat1 = [0 0 1]'; 
q1      = [0 0 0]';
vhat1   = -cross(omghat1,q1);
S1      = [omghat1;vhat1];
S1mat   = VecTose3(S1);
% S2 revolute
omghat2 = [1 0 0]'; 
q2      = [0 0 L1]';
vhat2   = -cross(omghat2,q2);
S2      = [omghat2;vhat2];
S2mat   = VecTose3(S2);
% S3 prismatic
omghat3 = [0 0 0]'; 
q3      = [0 0 0]';
vhat3   = [0 1 0]';
S3      = [omghat3;vhat3];
S3mat   = VecTose3(S3);
% joints 4 5 6 constitute a spherical wrist
% S4 revolute
omghat4 = [0 1 0]'; 
q4      = [0 0 L1]';
vhat4   = -cross(omghat4,q4);
S4      = [omghat4;vhat4];
S4mat   = VecTose3(S4);
% S5 revolute
omghat5 = [1 0 0]'; 
q5      = [0 0 L1]';
vhat5   = -cross(omghat5,q5);
S5      = [omghat5;vhat5];
S5mat   = VecTose3(S5);
% S6 revolute
omghat6 = [0 1 0]'; 
q6      = [0 0 L1]';
vhat6   = -cross(omghat6,q6);
S6      = [omghat6;vhat6];
S6mat   = VecTose3(S6);

% plot manipulator in home config
figure(1)
x0=200;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=600;  % width of figure
height=500; % height of figure
set(gcf,'position',[x0,y0,width,height])
grid on
volume_length =L1+L2;
plotvol([-(L2+L3+L6) (L2+L3+L6) -(L2+L3+L6+0.1) (L2+L3+L6+0.1) -L1 (L1+L2+L3+L6) ]);
hold on
view([71,48])
trplot(M1, 'length', 0.1)
plot3([M1(1,4) M2(1,4)],[M1(2,4) M2(2,4)],[M1(3,4) M2(3,4)], 'k--', 'linewidth', 1)
plot3([M2(1,4) M3(1,4)],[M2(2,4) M3(2,4)],[M2(3,4) M3(3,4)], 'k--',  'linewidth', 1)
plot3([M3(1,4) M456(1,4)],[M3(2,4) M456(2,4)],[M3(3,4) M456(3,4)], 'k--',  'linewidth', 1)
plot3([M456(1,4) M(1,4)],[M456(2,4) M(2,4)],[M456(3,4) M(3,4)], 'k--',  'linewidth', 1)
plot_sphere([M2(1:3,4)], 'color','k', 0.01)
plot_sphere([M3(1:3,4)], 'color','k',0.01)
plot_sphere([M456(1:3,4)], 'color','k',0.01)
trplot(M, 'length', 0.1)
plot_sphere([M(1:3,4)], 'color','k',0.01)
hold on
% the FK for each joint is:
theta1 = deg2rad(10); % [rad]
theta2 = deg2rad(10); % [rad]
theta3 = 0.; % [m] because this is a prismatic joint
theta4 = deg2rad(10); % [rad]
theta5 = deg2rad(10); % [rad]
theta6 = deg2rad(10); % [rad]
Tse = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*MatrixExp6(S4mat*theta4)*MatrixExp6(S5mat*theta5)*MatrixExp6(S6mat*theta6)*M;
Ts1 = MatrixExp6(S1mat*theta1)*eye(4);
Ts2 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*M2;
Ts3 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*M3;
Ts4 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*MatrixExp6(S4mat*theta4)*M456;
Ts5 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*MatrixExp6(S4mat*theta4)*MatrixExp6(S5mat*theta5)*M456;
Ts6 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*MatrixExp6(S4mat*theta4)*MatrixExp6(S5mat*theta5)*MatrixExp6(S6mat*theta6)*M456;
%plot again after motion
plot3([M1(1,4) Ts2(1,4)],[M1(2,4) Ts2(2,4)],[M1(3,4) Ts2(3,4)], 'k-', 'linewidth', 2)
plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'k-',  'linewidth', 2)
plot3([Ts3(1,4) Ts6(1,4)],[Ts3(2,4) Ts6(2,4)],[Ts3(3,4) Ts6(3,4)], 'k-',  'linewidth', 2)
plot3([Ts6(1,4) Tse(1,4)],[Ts6(2,4) Tse(2,4)],[Ts6(3,4) Tse(3,4)], 'k-',  'linewidth', 2)
plot_sphere([Ts2(1:3,4)], 0.01)
plot_sphere([Ts3(1:3,4)], 0.01)
plot_sphere([Ts6(1:3,4)], 0.01)
trplot(Tse, 'length', 0.1)
plot_sphere([Tse(1:3,4)], 0.01)



% from here we start the IK
% set desired position and orientation of the end-effector as a desired HTM
% wrt the base frame. Some possible configuration are listed below as
% comments:
%Tsd = RpToTrans(eye(3), [0.2 0.2 0.2]');
Tsd  =RpToTrans(roty(-pi), [0.2 0.1 0]')
% or I can use the solution of the FK above as the input for the IK so that
% I can be sure it has a solution
%Tsd = Tse
figure(1)
plot_sphere([Tsd(1,4) Tsd(2,4) Tsd(3,4)],0.02, 'color', 'r')
trplot(Tsd, 'color','r', 'length', 0.1)

% the follwing commands initialize the body twist before we enter the
% Newton Raphson algorithm:
% compute the desired HTM wrt the current end-effector pose Tsb  = M
Tbd = TransInv(Tse)*Tsd; % desired HTM of the end-effector
Vbmat = MatrixLog6(Tbd); % desired skew-symmetric twist of the end-effector
Vb = se3ToVec(Vbmat);
% extracts omega and v of the twist so that they can be used to estimate
% the error 
omgb = Vb(1:3)+1000;
vb   = Vb(4:6)+1000;
% creates a vector of initial values of joint-coordinates
theta_list = [0 0 0 0 0 0]';
% defines the tolerance within which the compueted solution is acceptable
tolerance = 0.001;
i = 0;
figure(2)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=600;  % width of figure
height=500; % height of figure
set(gcf,'position',[x0,y0,width,height])
grid on
volume_length =L1+L2;
plotvol([-(L2+L3+L6) (L2+L3+L6) -(L2+L3+L6+0.1) (L2+L3+L6+0.1) -L1 (L1+L2+L3+L6) ]);
hold on
view([71,48])
plot_sphere([Tsd(1,4) Tsd(2,4) Tsd(3,4)],0.02, 'color', 'r')
while norm(omgb)>tolerance || norm(vb)>tolerance
    i = i+1 % update loop counter
    % following commands compute the FK at each iteration of the loop, this
    % is needed to compute the Jacobian and the current pose of the
    % end-effector in order to estimate how big the error is.
    Ts1 = MatrixExp6(S1mat*theta_list(1));
    Ts2 = MatrixExp6(S1mat*theta_list(1))*MatrixExp6(S2mat*theta_list(2));
    Ts3 = MatrixExp6(S1mat*theta_list(1))*MatrixExp6(S2mat*theta_list(2))*MatrixExp6(S3mat*theta_list(3));
    Ts4 = MatrixExp6(S1mat*theta_list(1))*MatrixExp6(S2mat*theta_list(2))*MatrixExp6(S3mat*theta_list(3))*MatrixExp6(S4mat*theta_list(4));
    Ts5 = MatrixExp6(S1mat*theta_list(1))*MatrixExp6(S2mat*theta_list(2))*MatrixExp6(S3mat*theta_list(3))*MatrixExp6(S4mat*theta_list(4))*MatrixExp6(S5mat*theta_list(5));
    Ts6 = MatrixExp6(S1mat*theta_list(1))*MatrixExp6(S2mat*theta_list(2))*MatrixExp6(S3mat*theta_list(3))*MatrixExp6(S4mat*theta_list(4))*MatrixExp6(S5mat*theta_list(5))*MatrixExp6(S6mat*theta_list(6));
    % compute the body Twist of the end-effector as the following equation:
    % Ts6 * M = Tsb is the current end-effector HTM
    % TransInv(Tsb) = Tbs
    % Tbs*Tsd = Tbd expresses the desired position of the end-effector wrt
    % to the current one, basically this says how much the end-effector
    % needs to be moved from the current state in order to reach the
    % desired state.
    % MatrixLog6(Tbd) = [Vb] computes the skew-symmetric form of the body
    % twist needed to move the end-effector from Tsb to Tsd
    Vb = se3ToVec(MatrixLog6(TransInv(Ts6*M)*Tsd))
    % computes the space Jacobian of the end-effector
    Js = [S1 Adjoint(Ts1)*S2 Adjoint(Ts2)*S3 Adjoint(Ts3)*S4 Adjoint(Ts4)*S5 Adjoint(Ts5)*S6];
    % transforms the space Jacobian into the Body jacobian
    Jb = Adjoint(TransInv(Ts6*M))*Js;
    % computes the corrected joint-coord list according to the
    % Newton-Raphson algorithm
    theta_list = theta_list+pinv(Jb)*Vb;
    % computes the rotation and linear velcity part of the newly estimated
    % body twist which are going to be used as an estimate of the distance
    % from the current state of the end-effector and the desired one:
    % basically they are used as the error estimator
    omgb = Vb(1:3);
    vb = Vb(4:6);

end

% computes the position and pose of the joints and end-effector and plots
M2 = Ts2*M2;
M3 = Ts3*M3;
M456 = Ts6*M456;
M = Ts6*M;
figure(2)
link1 = plot3([M1(1,4) M2(1,4)],[M1(2,4) M2(2,4)],[M1(3,4) M2(3,4)], 'k-', 'linewidth', 2);
link2 = plot3([M2(1,4) M3(1,4)],[M2(2,4) M3(2,4)],[M2(3,4) M3(3,4)], 'k-',  'linewidth', 2);
link3 = plot3([M3(1,4) M456(1,4)],[M3(2,4) M456(2,4)],[M3(3,4) M456(3,4)], 'k-',  'linewidth', 2);
link4 = plot3([M456(1,4) M(1,4)],[M456(2,4) M(2,4)],[M456(3,4) M(3,4)], 'k-',  'linewidth', 2);
sphere1 = plot_sphere([M2(1:3,4)], 0.01);
sphere2 = plot_sphere([M3(1:3,4)], 0.01);
sphere3 = plot_sphere([M456(1:3,4)], 0.01);
trplot(M, 'length', 0.1);
sphere4 = plot_sphere([M(1:3,4)], 0.01);
trplot(Tsd, 'color','r', 'length', 0.05)

