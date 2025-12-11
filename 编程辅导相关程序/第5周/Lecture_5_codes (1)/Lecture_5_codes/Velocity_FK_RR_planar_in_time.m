
clear all
close all
format short

L1 = 0.5;
L2 = 0.4;

volume_length = 1.5;
figure(1) % in this figure we plot the resultant end-effector velocity from the solution of the velocity FK
x0=100;     % location of figure in the screen
y0=100;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(volume_length);
hold on
view(0,90)
title(strcat('velocity of the end effector $\dot {\vec x_e}$'),'interpreter','latex')


% assign joint coordinates at initial condition
theta1 = deg2rad(0);
theta2 = deg2rad(0);
% assign joint velocities profile (constant or time-varying: if time-varying, this command goes into the loop)
dot_theta1 = deg2rad(20); % deg/sec 
dot_theta2 = deg2rad(30); % deg/sec 
% assing time variables
t0 = 0.0; % [sec]
tend = 20.0; % [sec]
Deltat = 0.25;
N = ceil((tend-t0)/Deltat); % number of interval during the motion where you want to compute the FK

for i=1:N
    % if joint velocites vary in time, you can specify them here:
    dot_theta1 = 0.2*sin(0.1*i*Deltat); % deg/sec 
    dot_theta2 = 0.5*sin(0.5*i*Deltat); % deg/sec 
    % updates joint coordinates values after Deltat
    theta1 = theta1+dot_theta1*Deltat;
    theta2 = theta2+dot_theta2*Deltat;
    % computes FK
    Rs0 = rotz(theta1); % rotation of joint-1 wrt base
    ds0 = [0;0;0];      % displacement of joint-1 wrt base
    R01 = rotz(theta2); % rotation of joint-2 wrt base
    d01 = [L1;0; 0];    % displacement of joint-2 wrt base
    R12 = eye(3);       % <- this command simply says that the end-effector has the same orientation of the previous joint
    d12 = [L2; 0; 0];   % <- the end-effector is displaced L2 from joint-2 by means of the third link of the manipulator
    Ts0 = RpToTrans(Rs0,ds0);
    T01 = RpToTrans(R01, d01);
    T12 = RpToTrans(R12, d12); 
    Ts1 = Ts0*T01; 
    Ts2 = Ts1*T12; 
    % express the end-effector in vector form
    x_e  = HTM2Vec(Ts2);
    % compute the Jacobian in the current configuration
    J = [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2);...
     L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2)];
    % build the joint velocities vector
    dot_theta = [dot_theta1;dot_theta2];
    % compute the Forward Velocity Kinematics at the current configuration
    dot_x_e = J*dot_theta;
    
    figure(1)
    plotvol(volume_length);
    hold on
    view(0,90)
    title(strcat('velocity of the end effector $\dot {\vec x_e}$ at time $\;$', num2str(Deltat*i), ' [s]'),'interpreter','latex')
%     trplot(eye(4),'frame', 's', 'length', L2/2)
%     trplot(Ts1, 'frame', '1','color','r', 'length', L2/2)
%     trplot(Ts2,'frame', '2','color','r','length', L2/2)
    plot3([0 Ts1(1,4)], [0 Ts1(2,4)], [0 Ts1(3,4)], 'k')
    plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k')
    plot_sphere([0  0 0], L2/15)
    plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
    plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
    % plot the end-effector velocity
    plot_arrow(x_e(1:2),x_e(1:2)+dot_x_e(1:2,1),'r', 0.4)
    drawnow
    pause(0.01)
    hold off

    figure(2)
    x0=500;     % location of figure in the screen
    y0=100;     % location of figure in the screen
    width=400;  % width of figure
    height=400; % height of figure
    set(gcf,'position',[x0,y0,width,height])    % set figures specifications
    grid on
    hold on
    plot(Deltat*i, dot_theta1, '.b')
    plot(Deltat*i, dot_theta2, '.r')
    legend('joint-1', 'joint-2')
    xlabel('time [s]')
    ylabel('joint velocities [rad/s]')
    title(strcat('velocity of the joints $\dot {\vec \theta}$'),'interpreter','latex')

    figure(3)
    x0=1000;     % location of figure in the screen
    y0=100;     % location of figure in the screen
    width=400;  % width of figure
    height=400; % height of figure
    set(gcf,'position',[x0,y0,width,height])    % set figures specifications
    grid on
    hold on
    plot(Deltat*i, dot_x_e(1), '.b')
    plot(Deltat*i, dot_x_e(2), '.r')
    legend('x velocity', 'y velocity')
    xlabel('time [s]')
    ylabel('end-effector velocities [m/s]')
    title(strcat('velocity of the end-effector $\dot {\vec x_e}$'),'interpreter','latex')



    
end