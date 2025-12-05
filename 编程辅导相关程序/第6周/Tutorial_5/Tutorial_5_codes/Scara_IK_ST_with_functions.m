% scara robot IK with Screw Theory

clear all
close all
format short

L1 = 0.5;
L11 = 0.1;
L2 = 0.5;
L3 = 0.2;
L4 = 0.2
theta1 = 0; % [rad]
theta2 = 0; % [rad]
theta3 = 0; % [m]
% define joint positions and orientation at home config
Ts = eye(4);
M = RpToTrans(roty(pi), [0 L2+L3 L1-L4]');
M1 = RpToTrans(eye(3), [0 0 L1]');
M11 = RpToTrans(eye(3), [0 L11 L1]');
M2 = RpToTrans(eye(3), [0 L2 L1]');
M3 = RpToTrans(roty(pi), [0 L2+L3 L1]');

% plot manipulator in home config
figure(1)
view(160, 40)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol([-(L2+L3) (L2+L3) -(L2+L3+0.1) (L2+L3+0.1) 0 (L1+L2+L3) ]);
trplot(Ts, 'length', 0.1)
plot3([Ts(1,4) M1(1,4)],[Ts(2,4) M1(2,4)],[Ts(3,4) M1(3,4)], 'k-', 'linewidth', 2)
plot3([M1(1,4) M11(1,4)],[M1(2,4) M11(2,4)],[M1(3,4) M11(3,4)], 'k-', 'linewidth', 2)
plot3([M11(1,4) M2(1,4)],[M11(2,4) M2(2,4)],[M11(3,4) M2(3,4)], 'k-', 'linewidth', 2)
plot3([M2(1,4) M3(1,4)],[M2(2,4) M3(2,4)],[M2(3,4) M3(3,4)], 'k-',  'linewidth', 2)
plot3([M3(1,4) M(1,4)],[M3(2,4) M(2,4)],[M3(3,4) M(3,4)], 'k-',  'linewidth', 2)
plot_sphere([M1(1:3,4)],0.03,  'color','b')
plot_sphere([M2(1:3,4)], 0.03,  'color','b')
plot_sphere([M3(1:3,4)], 0.03,  'color','b')
plot_sphere([M11(1:3,4)], 0.03,  'color','b')
trplot(M, 'length', 0.2)
plot_sphere([M(1:3,4)], 0.03,  'color','b')
hold on

% define screw axes at home configuration
% S1 recvolute
omghat1 = [0 0 1]'; 
q1      = [0 0 L1]';
vhat1   = -cross(omghat1,q1);
S1      = [omghat1;vhat1];
S1mat   = VecTose3(S1);
% S11 recvolute
omghat11 = [0 0 0]'; 
q11      = [0 0 0]';
vhat11   = [0 1 0]';
S11      = [omghat11;vhat11];
S11mat   = VecTose3(S11);
% S2 revolute
omghat2 = [0 0 1]'; 
q2      = [0 L2 L1]';
vhat2   = -cross(omghat2,q2);
S2      = [omghat2;vhat2];
S2mat   = VecTose3(S2);
% S3 revolute
omghat3 = [0 0 0]'; 
q3      = [0 0 0]';
vhat3   = [0 0 -1]';
S3      = [omghat3;vhat3];
S3mat   = VecTose3(S3);

Slist = [S1 S11 S2 S3]

Tsd = RpToTrans(roty(pi)*rotx(0), [L3 L2  0]');
%Tsd = RpToTrans(rotx(pi/20)*roty(pi), [0.1 0.1 0.3]');
plot_sphere([Tsd(1:3,4)], 0.03,  'color','r');

% IK
thetalist0 = [0 0 0 0]';
tolerance = 0.001;
[thetalist, success] = IKinSpace(Slist, M, Tsd, thetalist0, tolerance, tolerance)
Tse = FKinSpace(M, Slist, thetalist)
Ts1 = M1;
Ts11 = FKinSpace(M11, Slist(:,1), thetalist(1));
Ts2 = FKinSpace(M2, Slist(:,1:2), thetalist(1:2));
Ts3 = FKinSpace(M3, Slist(:,1:3), thetalist(1:3));
figure(1)
view(160, 40)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plot3([Ts(1,4) Ts1(1,4)],[Ts(2,4) Ts1(2,4)],[Ts(3,4) Ts1(3,4)], 'k-', 'linewidth', 2)
plot3([Ts1(1,4) Ts11(1,4)],[Ts1(2,4) Ts11(2,4)],[Ts1(3,4) Ts11(3,4)], 'k-', 'linewidth', 2)
plot3([Ts11(1,4) Ts2(1,4)],[Ts11(2,4) Ts2(2,4)],[Ts11(3,4) Ts2(3,4)], 'k-', 'linewidth', 2)
plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'k-',  'linewidth', 2)
plot3([Ts3(1,4) Tse(1,4)],[Ts3(2,4) Tse(2,4)],[Ts3(3,4) Tse(3,4)], 'k-',  'linewidth', 2)
plot_sphere([Ts1(1:3,4)],0.03,  'color','k')
plot_sphere([Ts2(1:3,4)], 0.03,  'color','k')
plot_sphere([Ts3(1:3,4)], 0.03,  'color','k')
plot_sphere([Ts11(1:3,4)], 0.03,  'color','k')
plot_sphere([Tse(1:3,4)], 0.03,  'color','k')
trplot(Tsd, 'color', 'r', 'length', 0.2)
trplot(Tse, 'color', 'k', 'length', 0.1)