% RR planar manipulator with PoE
clear all
close all

% defines links geometry
L1 = 0.2;
L2 = 0.15;

% define HTMs of the base frame and end-effector frame in the home config
I = eye(3);
T0 = RpToTrans(I,[0 0 0]'); 
M  = RpToTrans(I,[L1+L2 0 0]');
% define the q vectors for joint 1 and 2 in the home config
q1 = [0 0 0]';
q2 = [L1 0 0]';
% define unit axis of rotation for joints 1 and 2
omg1 = [0 0 1];
omg2 = [0 0 1];
% define pitch of joints 1 and 2: because the joints are rotational joint,
% the pitch is assumed =0
h1 = 0;
h2 = 0;
% compute the Screw axes for joint 1 and 2
S1 = [omg1'; (-cross(omg1,q1)+h1*omg1)'];
S2 = [omg2'; (-cross(omg2,q2)+h2*omg2)'];
% transform the vector form of the screw axes in skew-symmetric matrix form
S1_mat = VecTose3(S1)
S2_mat = VecTose3(S2)
% define how the joint angles should be rotated: you can modify this
theta1 = deg2rad(10)
theta2 = deg2rad(30)
% compute the HTM of the end-effector based on Product of Exponentials
T0e = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*M;
% plot base frame and end-effector frame
figure(1)
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(3*L2)
view(0,90)
trplot(T0, 'length', L1/2, 'color', 'k')
hold on
trplot(T0e,'length', L1/2, 'color', 'r')
% if I want to visulize and plot also the other links (beside the base and
% end-effector frame), I need to also define an initial HTM for frames 1,
% 2, etc in the home configuration and comute the PoE for all of them

 M1 = RpToTrans(I,[0 0 0]'); % HTM attached to joint 1 in home config
 M2 = RpToTrans(I,[L1 0 0]');% HTM attached to joint 2 in home config
 T01 = MatrixExp6(S1_mat*theta1)*M1; % find HTM of joint 1 after rotation using PoE
 T02 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*M2; % find HTM of joint 2 after rotation using PoE
 figure(1)
 x0=300;     % location of figure in the screen
 y0=200;     % location of figure in the screen
 width=400;  % width of figure
 height=400; % height of figure
 set(gcf,'position',[x0,y0,width,height])    % set figures specifications
 trplot(T01, 'length', L1/2)
 hold on
 trplot(T02,'length', L1/2)
 plot3([T01(1,4) T02(1,4)], [T01(2,4) T02(2,4)], [T01(3,4) T02(3,4)],'o-k','linewidth',1 )
 plot3([T02(1,4) T0e(1,4)], [T02(2,4) T0e(2,4)], [T02(3,4) T0e(3,4)],'o-k','linewidth',1 )


