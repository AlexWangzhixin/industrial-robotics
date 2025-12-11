% this program computes the space jacobian to solve the forward velocity kinematics
% of an RRRRR planar manipulator.
% the program works like this:
% 1) first solves the FK with Product of Exponentials by finding
% the Screw Axes in the home configuration (joint angles = 0)
% 2) then computes the Space Jacobian by using the Adjoint function, that
% means that column 1 of the Jacobian is equal to S1, column 2 of the
% Jacobian is Adjiont_T01(S2), column 3 of the Jacobian is Adjoint_T02(S3),so: 
% Js = [Js1 Js2 Js3 Js4 Js5] = [S1 Adjoint_T01(S2) Adjoint_T02(S3) Adjoint_T03(S4) Adjoint_T04(S5)]
% 3) solves the forward velocity kinematics in the spatial twist Vs = Js*dot_theta
% 4) to plot the velocities, it transforms the twist wrt body frame Vb = Adjoint_T05(Vs)
% and than takes the linear velocity components of Vd and rotates them wrt
% the base v_endeffector = R_endeffector*Vb(4:6)
clear all
close all
format short

Ts = eye(4);
% set angles in home configuration all joint angles = 0 degrees
theta1 = rad2deg(0);
theta2 = rad2deg(0);
theta3 = rad2deg(0);
theta4 = rad2deg(0);
theta5 = rad2deg(0);

L1 = 1;
L2 = 1;
L3 = 1;
L4 = 1;
L5 = 1;
% the following are the HTM of the end-effector and of each link in the 
% home configuration and we define them just so that we can visualize 
% the whole manipulator in the plot
M = RpToTrans(eye(3), [L1+L2+L3+L4+L5 0 0]');
M1 = RpToTrans(eye(3), [0 0 0]');
M2 = RpToTrans(eye(3), [L1 0 0]');
M3 = RpToTrans(eye(3), [L1+L2 0 0]');
M4 = RpToTrans(eye(3), [L1+L2+L3 0 0]');
M5 = RpToTrans(eye(3), [L1+L2+L3+L4 0 0]');
% now we define all the Screw Axis in the home configuration, just like we
% normally do for the FK with PoE
omghat1 = [0 0 1]';
omghat2 = [0 0 1]';
omghat3 = [0 0 1]';
omghat4 = [0 0 1]';
omghat5 = [0 0 1]';
q1 = [0 0 0]';
q2 = [L1 0 0]';
q3 = [L1+L2 0 0]';
q4 = [L1+L2+L3 0 0]';
q5 = [L1+L2+L3+L4 0 0]';
vhat1 = -cross(omghat1,q1);
vhat2 = -cross(omghat2,q2);
vhat3 = -cross(omghat3,q3);
vhat4 = -cross(omghat4,q4);
vhat5 = -cross(omghat5,q5);
S1 = [omghat1;vhat1];
S2 = [omghat2;vhat2];
S3 = [omghat3;vhat3];
S4 = [omghat4;vhat4];
S5 = [omghat5;vhat5];
% transform the Screw Axes in skew-symmetric form in order to compute PoE
S1mat = VecTose3(S1);
S2mat = VecTose3(S2);
S3mat = VecTose3(S3);
S4mat = VecTose3(S4);
S5mat = VecTose3(S5);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%         solve the FK at an instantaneous configuration 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% assign new joint angles
theta1 = deg2rad(10);
theta2 = deg2rad(20);
theta3 = deg2rad(30);
theta4 = deg2rad(-40);
theta5 = deg2rad(-10);
% compute end-effector HTM
Tse = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)...
    *MatrixExp6(S4mat*theta4)*MatrixExp6(S5mat*theta5)*M;
% compute all other HTM for plotting purposes
M1e = MatrixExp6(S1mat*theta1)*M1;
M2e = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*M2;
M3e = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*M3;
M4e = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)...
    *MatrixExp6(S4mat*theta4)*M4;
M5e = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)...
    *MatrixExp6(S4mat*theta4)*MatrixExp6(S5mat*theta5)*M5;
% plot the manipulator
figure(1)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=500;  % width of figure
height=600; % height of figure
set(gcf,'position',[x0,y0,width,height]) 
hold on
%title(strcat('velocity of the end effector $\dot {\vec x_e}$'),'interpreter','latex')
%plotvol(10)
view([0,90])
axis equal
grid on
hold on
trplot(Ts, 'arrow')
plot3([Ts(1,4) M1e(1,4)], [Ts(2,4) M1e(2,4)], [Ts(3,4) M1e(3,4)], 'k')
trplot(Tse, 'color', 'k', 'length', 0.2, 'arrow')
plot3([M1e(1,4) M2e(1,4)], [M1e(2,4) M2e(2,4)], [M1e(3,4) M2e(3,4)], 'k')
trplot(M1e, 'color', 'k', 'length', 0.2, 'arrow')
plot3([M2e(1,4) M3e(1,4)], [M2e(2,4) M3e(2,4)], [M2e(3,4) M3e(3,4)], 'k')
trplot(M2e, 'color', 'k', 'length', 0.2, 'arrow')
plot3([M3e(1,4) M4e(1,4)], [M3e(2,4) M4e(2,4)], [M3e(3,4) M4e(3,4)], 'k')
trplot(M3e, 'color', 'k', 'length', 0.2, 'arrow')
plot3([M4e(1,4) M5e(1,4)], [M4e(2,4) M5e(2,4)], [M4e(3,4) M5e(3,4)], 'k')
trplot(M4e, 'color', 'k', 'length', 0.2, 'arrow')
plot3([M5e(1,4) Tse(1,4)], [M5e(2,4) Tse(2,4)], [M5e(3,4) Tse(3,4)], 'k')
trplot(M5e, 'color', 'k', 'length', 0.2, 'arrow')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%          Compute Jacobian and forward velocity kinematics 
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute Jacobian matrix by using the Adjoint operator, so that 
% Jsi = Adjiont_T0i-1(Si) where 
% Adjiont_T0i-1 is the adjoint of HTM from frame 0 to frame i-1 
% and frame i is the frame where the screw axis Si is, see lecture slides
% compute first column of the jacobian
Js1 = S1;
% compute second column of the jacobian
% first compute the HTM from base to joint-2
Ts2 = MatrixExp6(S1mat*theta1);
% compute the Adjoint from 1 to 2
Adjoint2 = Adjoint(Ts2);
% Apply the Adjoint to Screw Axis 2
Js2 = Adjoint2*S2
% you can compare the use of Adjoint2 to comput Js2 with the analytic calculation
% where you estimate S2 when joint-1 is rotated of an amount theta1:  
%q2_analytic = [L1*cos(theta1) 0 0]'; %q2 vector, connecting base frame and joint2 when joint1 is rotated theta1
%Js2_analytic = [omghat2; -cross(omghat2,q2)] % analytic calculation of Js2 using q2

% compute third column of the Jacobian
Ts3 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2);
Adjoint3 = Adjoint(Ts3);
Js3 = Adjoint3*S3;

% compute fourth column of the Jacobian
Ts4 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3);
Adjoint4 = Adjoint(Ts4);
Js4 = Adjoint4*S4;

% compute fifth column of the Jacobian
Ts5 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)...
    *MatrixExp6(S4mat*theta4);
Adjoint5 = Adjoint(Ts5);
Js5 = Adjoint5*S5;

% assemble the space Jacobian matrix
Js = [Js1 Js2 Js3 Js4 Js5]

% compute the Forward velocity kinematics in the twist
% assign joint velocites
dot_theta1 = 0.1; % rad/sec
dot_theta2 = 0; % rad/sec
dot_theta3 = 0; % rad/sec
dot_theta4 = 0; % rad/sec
dot_theta5 = 0.4; % rad/sec
% assemble all joint velocities in a vector
dot_theta = [dot_theta1 dot_theta2 dot_theta3 dot_theta4 dot_theta5]'; 
% calculate the twist
Vs =Js*dot_theta;

% to plot the velocity acting on the end-effector, we
% need to transform the linear velocity of the space twist in the end-effector
% frame, to do this we take the HTM from the base to the end-effector T_se, then
% we do the inverse T_es
Tes = TransInv(MatrixExp6(S1mat*theta1)...
    *MatrixExp6(S2mat*theta2)...
    *MatrixExp6(S3mat*theta3)...
    *MatrixExp6(S4mat*theta4)...
    *MatrixExp6(S5mat*theta5)*M);
% then we compute the Adjoint_T_es, which allows to re-express a twist from
% the base frame to the end effector frame
AdjointTes = Adjoint(Tes);
% we apply the inverse of the Adjoint to the Jacobian and achieve the body
% jacobian
%Jb = AdjointTes*Js;
% and compute the twist in the end-effector 
%Vb = Jb*dot_theta
% alternatively we can simply use the Adjoint to compute Vb from Vs:
Vb = AdjointTes*Vs
% then we extract the linear velocities from the twist in the end-effector
vb = Vb(4:6);
% to plot this velocity we want to express the velocity of the end-effector
% wrt the base frame, so we extract the rotation of the end-effector
[Re,de] = TransToRp(Tse);
% apply the rotation to the velocity vector
vb_s = Re*vb
figure(1)
xlim([-1 5])
ylim([-5 5])
%plot_arrow(Tse(1:3,4), Tse(1:3,4)+vb, 'b-.' );
hold on
plot_arrow(Tse(1:3,4), Tse(1:3,4)+vb_s, 'r-.' );
plot_arrow(0*Tse(1:3,4), 0*Tse(1:3,4)+Vs(4:6), 'g-.' );
title(strcat('$\dot {\theta}_1= $', num2str(dot_theta1), ', ', ' $ \dot {\theta}_2= $', num2str(dot_theta2), ', ', ' $ \dot {\theta}_3= $', num2str(dot_theta3), ', ', ' $ \dot {\theta}_4= $', num2str(dot_theta4), ', ', ' $ \dot {\theta}_5= $ ', num2str(dot_theta5), ' [rad/sec]'),'interpreter','latex')


