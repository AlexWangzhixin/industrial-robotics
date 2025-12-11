% scara robot IK with Screw Theory

clear all
close all
format short

L1 = 0.5;
L2 = 0.1;
L3 = 0.2;
L4 = 0.2;
L5 = 0.2
theta1 = 0; % [rad]
theta2 = 0; % [m]
theta3 = 0; % [rad]
theta4 = 0; % [rad]
% define joint positions and orientation at home config
Ts = eye(4);
M = RpToTrans(roty(pi), [0 L2+L3+L4 L1-L5]');
M1 = RpToTrans(eye(3), [0 0 L1]');
M2 = RpToTrans(eye(3), [0 L2 L1]');
M3 = RpToTrans(eye(3), [0 L2+L3 L1]');
M4 = RpToTrans(roty(pi), [0 L2+L3+L4 L1]');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot manipulator in home config
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 figure(1)
 plotvol([-(L2+L3+L4+0.1) (L2+L3+L4+0.1) -(L2+L3+L4+0.1) (L2+L3+L4+0.1) -0.1 (L1+L2+L3) ]);
 view(-132, 25)
x0=100;     % location of figure in the screen
y0=100;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
 trplot(Ts, 'length', 0.1)
plot3([Ts(1,4) M1(1,4)],[Ts(2,4) M1(2,4)],[Ts(3,4) M1(3,4)], 'k-', 'linewidth', 2)
plot3([M1(1,4) M2(1,4)],[M1(2,4) M2(2,4)],[M1(3,4) M2(3,4)], 'k-', 'linewidth', 2)
plot3([M2(1,4) M3(1,4)],[M2(2,4) M3(2,4)],[M2(3,4) M3(3,4)], 'k-', 'linewidth', 2)
plot3([M3(1,4) M4(1,4)],[M3(2,4) M4(2,4)],[M3(3,4) M4(3,4)], 'k-',  'linewidth', 2)
plot3([M4(1,4) M(1,4)],[M4(2,4) M(2,4)],[M4(3,4) M(3,4)], 'k-',  'linewidth', 2)
plot_sphere([M1(1:3,4)],0.03,  'color','b')
plot_sphere([M2(1:3,4)], 0.03,  'color','b')
plot_sphere([M3(1:3,4)], 0.03,  'color','b')
plot_sphere([M4(1:3,4)], 0.03,  'color','b')
trplot(M, 'length', 0.2)
plot_sphere([M(1:3,4)], 0.03,  'color','b')
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% define screw axes at home configuration
% S1 recvolute
omghat1 = [0 0 1]'; 
q1      = [0 0 0]';
vhat1   = -cross(omghat1,q1);
S1      = [omghat1;vhat1];
S1mat   = VecTose3(S1);
% S2 prismatic
omghat2 = [0 0 0]'; 
q2      = [0 0 0]';
vhat2   = [0 1 0]';
S2      = [omghat2;vhat2];
S2mat   = VecTose3(S2);
% S3 revolute
omghat3 = [0 0 1]'; 
q3      = [0 L2+L3 0]';
vhat3   = -cross(omghat3,q3);
S3      = [omghat3;vhat3];
S2mat   = VecTose3(S2);
% S4 prismatic
omghat4 = [0 0 0]'; 
q4      = [0 0 0]';
vhat4   = [0 0 -1]';
S4      = [omghat4;vhat4];
S4mat   = VecTose3(S4);

Slist = [S1 S2 S3 S4];

% IK
Xstart = RpToTrans(roty(pi), [-0.4 -0.2  0.3]');
Xend = RpToTrans(roty(pi), [-0.4 0.3  0.5]');
Tf = 2;
N = 20;
deltat=Tf/(N-1);
method = 3;
traj1 = CartesianTrajectory(Xstart,Xend, Tf, N, method);
thetalist0 = [0 0 0 0]';
thetalist = [0 0 0 0]';
tolerance = 0.001;
for i=1:N
Tsd = cell2mat(traj1(i));
time(i) = i*deltat;
thetalist1 = thetalist;
[thetalist, success] = IKinSpace(Slist, M, Tsd, thetalist0, tolerance, tolerance);
thetalist2 = thetalist;
dot_thetalist(i, 1:4) = (thetalist2-thetalist1)/deltat;
thetalist0 = thetalist;
if i>1
    x = Tse(1,4);
    y = Tse(2,4);
    z = Tse(3,4); 
end
Tse = FKinSpace(M, Slist, thetalist)
Ts1 = M1;
Ts2 = FKinSpace(M2, Slist(:,1), thetalist(1));
Ts3 = FKinSpace(M3, Slist(:,1:2), thetalist(1:2));
Ts4 = FKinSpace(M4, Slist(:,1:3), thetalist(1:3));
xpos(i) = Tse(1,4);
ypos(i) = Tse(2,4);
zpos(i) = Tse(3,4);

if i>1
    dot_x(i) = (Tse(1,4)-x)/deltat;
    dot_y(i) = (Tse(2,4)-y)/deltat;
    dot_z(i) = (Tse(3,4)-z)/deltat;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot manipulator after IK
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1)
%plot_sphere([Tsd(1:3,4)], 0.03,  'color','r');
%trplot(Tsd, 'color', 'r', 'length', 0.1)
plot3([Ts(1,4) Ts1(1,4)],[Ts(2,4) Ts1(2,4)],[Ts(3,4) Ts1(3,4)], 'k-', 'linewidth', 1)
plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'k-', 'linewidth', 1)
plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'k-',  'linewidth', 1)
plot3([Ts3(1,4) Ts4(1,4)],[Ts3(2,4) Ts4(2,4)],[Ts3(3,4) Ts4(3,4)], 'k-',  'linewidth', 1)
plot3([Ts4(1,4) Tse(1,4)],[Ts4(2,4) Tse(2,4)],[Ts4(3,4) Tse(3,4)], 'k-', 'linewidth', 1)
plot_sphere([Ts1(1:3,4)],0.02,  'color','b')
plot_sphere([Ts2(1:3,4)], 0.02,  'color','b')
plot_sphere([Ts3(1:3,4)], 0.02,  'color','b')
plot_sphere([Ts4(1:3,4)], 0.02,  'color','b')
plot_sphere([Tse(1:3,4)], 0.02,  'color','r')
%trplot(Tse,'color','r', 'length', 0.05)
pause(0.1)
end

figure(2)
x0=500;     % location of figure in the screen
y0=100;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plot(time(3:end), dot_thetalist(3:end,1));
hold on
plot(time(3:end), dot_thetalist(3:end,2))
plot(time(3:end), dot_thetalist(3:end,3))
plot(time(3:end), dot_thetalist(3:end,4))
legend('$\dot\theta_1$', '$\dot\theta_2$', '$\dot\theta_3$', '$\dot\theta_4$','interpreter', 'latex')
xlabel('time [sec]','interpreter', 'latex')
ylabel('joint vspeed [rad/s] or [m]','interpreter', 'latex')
title('joint speed during trajectory','interpreter', 'latex')

figure(3)
x0=900;     % location of figure in the screen
y0=100;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plot(time(3:end), dot_x(3:end));
hold on
plot(time(3:end), dot_y(3:end))
plot(time(3:end), dot_z(3:end))
legend('$\dot x_e$', '$\dot y_e$', '$\dot z_e$','interpreter', 'latex')
xlabel('time [sec]','interpreter', 'latex')
ylabel('linear speed [m/s]','interpreter', 'latex')
title('end-effector speed during trajectory','interpreter', 'latex')

figure(4)
x0=1200;     % location of figure in the screen
y0=100;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plot(time(3:end), xpos(3:end));
hold on
plot(time(3:end), ypos(3:end))
plot(time(3:end), zpos(3:end))
legend('$x_e$', '$y_e$', '$z_e$','interpreter', 'latex')
xlabel('time [sec]','interpreter', 'latex')
ylabel('position [m]','interpreter', 'latex')
title('end-effector positions during trajectory','interpreter', 'latex')