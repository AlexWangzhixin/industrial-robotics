% FK example of an RRR serial spatial manipulator solved with PoE
% in this case the end-effector frame is not attached to joint 3, but it is
% displaced a distance L3 in the x direction with respect to it.

clear all
close all
format short

%define geometrical parameters
theta1=deg2rad(0);
theta2=deg2rad(0);
theta3=deg2rad(0);
L1 = 2;
L2 = 1;
L3 = 0.5;

%define base frame
T_s = eye(4);
%define end-effector frame wrt {s}
M = [0 0 1 L1+L3;  ...
     0 1 0 0;    ...
    -1 0 0 -L2;  ...
     0 0 0 1];
%for plotting purposes only, we can define the home config of joint-2
%Notice that we don't need this to only compute the end-effectore pose
R2 = rotx(pi/2)*rotz(-pi/2); % this command set the orientation of joint-2 with the z axis aligned with the rotation axis
d2 = [L1 0 0]';
M2 = RpToTrans(R2,d2); % this defines the HTM of joint-2 in the home configuation
M3 = [0 0 1 L1;  ...
     0 1 0 0;    ...
    -1 0 0 -L2;  ...
     0 0 0 1];
% plot the manipulator at its home configuration
volume_length = 4;
figure(1)
plotvol(volume_length)
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
view(37,33)
trplot(T_s)
hold on
plot3([0 T_s(1,4)],[0 T_s(2,4)],[-volume_length T_s(2,4)], 'o-k','linewidth',1)
plot3([T_s(1,4) M2(1,4)],[T_s(2,4) M2(2,4)],[T_s(3,4) M2(3,4)], 'o-k','linewidth',1)
plot3([M2(1,4) M3(1,4)],[M2(2,4) M3(2,4)],[M2(3,4) M3(3,4)], 'o-k','linewidth',1)
plot3([M3(1,4) M(1,4)],[M3(2,4) M(2,4)],[M3(3,4) M(3,4)], 'o-k','linewidth',1)
% trplot(T_s,'length', 0.2,'color','k')
% trplot(M2,'length', 0.2,'color','k')
% trplot(M,'length', 0.2,'color','k')
drawnow
pause(1.5)


%define the screw axes
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% S1
omg_hat_1 = [0 0 1]';
q1 = [0 0 0]';
v_hat_1 = -cross(omg_hat_1,q1);
S1 = [omg_hat_1;v_hat_1];
%compute the screw axis in skew symmetric form
S1mat = VecTose3(S1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% S2
omg_hat_2 = [0 -1 0]';
q2 = [L1 0 0]';
v_hat_2 = -cross(omg_hat_2,q2);
S2 = [omg_hat_2;v_hat_2];
%compute the screw axis in skew symmetric form
S2mat = VecTose3(S2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% S3
omg_hat_3 = [1 0 0]';
q3 = [0 0 -L2]';
v_hat_3 = -cross(omg_hat_3,q3);
S3 = [omg_hat_3;v_hat_3];
%compute the screw axis in skew symmetric form
S3mat = VecTose3(S3);

% set desired joint angles
theta1=deg2rad(20);
theta2=deg2rad(0);
theta3=deg2rad(40);
% solve the FK by multiplying the matrix exponential of the screw axes
T_s1 = MatrixExp6(S1mat*theta1)*T_s
T_s2 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*M2
T_s3 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*M3
T_se = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*M

% plot some stuff
figure(1)
hold on
plot3([0 T_s(1,4)],[0 T_s(2,4)],[-volume_length T_s(2,4)], 'o-r','linewidth',1)
plot3([T_s(1,4) T_s1(1,4)],[T_s(2,4) T_s1(2,4)],[T_s(3,4) T_s1(3,4)], 'o-r','linewidth',1)
plot3([T_s1(1,4) T_s2(1,4)],[T_s1(2,4) T_s2(2,4)],[T_s1(3,4) T_s2(3,4)], 'o-r','linewidth',1)
plot3([T_s2(1,4) T_s3(1,4)],[T_s2(2,4) T_s3(2,4)],[T_s2(3,4) T_s3(3,4)], 'o-r','linewidth',1)
plot3([T_s3(1,4) T_se(1,4)],[T_s3(2,4) T_se(2,4)],[T_s3(3,4) T_se(3,4)], 'o-r','linewidth',1)
% trplot(T_s1,'length', 0.2,'color','r')
% trplot(T_s2,'length', 0.2,'color','r')
% trplot(T_s3,'length', 0.2,'color','r')
trplot(T_se,'length', 0.2,'color','r')
