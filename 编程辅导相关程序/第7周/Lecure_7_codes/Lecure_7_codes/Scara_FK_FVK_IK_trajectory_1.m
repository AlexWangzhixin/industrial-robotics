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
%plotvol([-(L2+L3) (L2+L3) -(L2+L3+0.1) (L2+L3+0.1) 0 (L1+L2+L3) ]);
view(160, 40)
x0=200;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol([-(L2+L3+L4+0.1) (L2+L3+L4+0.1) -(L2+L3+L4+0.1) (L2+L3+L4+0.1) -0.1 (L1+L2+L3) ]);
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
% S1 revolute
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
tend = 5;
timesteps = 200;
deltat = tend/timesteps;
omegax = pi/2;
omegay = pi;
omegaz = pi;
thetalist0 = [0 0 0 0]';
tolerance = 0.001;
figure(1)
thetalist = [0 0 0 0]';
for i=1:timesteps
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % assign values of Tsd in time which define the desired trajectory
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time(i) = i*deltat;
    x = 0.2-0.4*sin(omegax*i*deltat);
    y = 0.2-0.1*sin(omegay*i*deltat);
    z = 0.2-0.2*sin(omegaz*i*deltat);
    Tsd = RpToTrans(roty(-pi), [x y z]');   
    thetalist1 = thetalist;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % compute inverse kinematic for each Tsd of the trajectory
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [thetalist, success] = IKinSpace(Slist, M, Tsd, thetalist0, tolerance, tolerance)
    thetalist2 = thetalist;
    dot_thetalist(i, 1:4) = (thetalist2-thetalist1)/deltat;
    thetalist0 = thetalist;
    Tse = FKinSpace(M, Slist, thetalist);
    Ts1 = M1;
    Ts2 = FKinSpace(M2, Slist(:,1), thetalist(1));
    Ts3 = FKinSpace(M3, Slist(:,1:2), thetalist(1:2));
    Ts4 = FKinSpace(M4, Slist(:,1:3), thetalist(1:3));

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % plot manipulator after IK
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    figure(1)
    view(160, 40)
    link1 = plot3([Ts(1,4) Ts1(1,4)],[Ts(2,4) Ts1(2,4)],[Ts(3,4) Ts1(3,4)], 'k-', 'linewidth', 2);
    link2 = plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'k-', 'linewidth', 2);
    link3 = plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'k-',  'linewidth', 2);
    link4 = plot3([Ts3(1,4) Ts4(1,4)],[Ts3(2,4) Ts4(2,4)],[Ts3(3,4) Ts4(3,4)], 'k-',  'linewidth', 2);
    link5 = plot3([Ts4(1,4) Tse(1,4)],[Ts4(2,4) Tse(2,4)],[Ts4(3,4) Tse(3,4)], 'k-', 'linewidth', 2);
    sphere1 = plot_sphere([Ts1(1:3,4)],0.02,  'color','k');
    sphere2 = plot_sphere([Ts2(1:3,4)], 0.02,  'color','k');
    sphere3 = plot_sphere([Ts3(1:3,4)], 0.02,  'color','k');
    sphere4 = plot_sphere([Ts4(1:3,4)], 0.02,  'color','k');
    sphere5 = plot_sphere([Tse(1:3,4)], 0.02,  'color','k');
    frame1 = trplot(Tse, 'length', 0.1);
    plot3(Tse(1,4), Tse(2,4), Tse(3,4), 'r.')
    drawnow
    pause(0.01)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % the commands below simply make sure to not remove the plot of the
        % manipulator at the last timestep of the calculation
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if i ~=timesteps
        delete(link1)
        delete(link2)
        delete(link3)
        delete(link4)
        delete(link5)
        delete(sphere1)
        delete(sphere2)
        delete(sphere3)
        delete(sphere4)
        delete(sphere5)
        delete(frame1)
        end  
end

figure(2)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=800;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plot(time(10:end), dot_thetalist(10:end,1));
hold on
plot(time(10:end), dot_thetalist(10:end,2))
plot(time(10:end), dot_thetalist(10:end,3))
plot(time(10:end), dot_thetalist(10:end,4))
legend('$\dot\theta_1$', '$\dot\theta_2$', '$\dot\theta_3$', '$\dot\theta_4$','interpreter', 'latex')
xlabel('time [sec]','interpreter', 'latex')
ylabel('joint speed [rad/s] or [m/s]','interpreter', 'latex')
title('joint speed during trajectory','interpreter', 'latex')
