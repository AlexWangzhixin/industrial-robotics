% Forward Velocity Kinematic RRRP
clear all
close all

L1 = 1;
L2 = 0.5;
L3 = 0.5;
L4 = 0.5;
I = eye(3);

% first we solve the FK with PoE
% we start by defining the Screw Axes in the home configuration and the
% HTMs in the home configuration
h1      = 0;
h2      = 0;
h3      = 0;
h4      = 1;
omghat1 = [0 0 1]';
omghat2 = [0 0 1]';
omghat3 = [0 0 1]';
omghat4 = [0 0 1]';
q1      = [0 0 0]';
q2      = [L1 0 0]';
q3      = [L1+L2 0 0]';
q4      = [0 0 0]';
vhat1   = [-cross(omghat1,q1)];
vhat2   = [-cross(omghat2,q2)];
vhat3   = [-cross(omghat3,q3)];
vhat4   = [omghat4*h4];
S1      = [omghat1;vhat1];
S2      = [omghat2;vhat2];
S3      = [omghat3;vhat3];
S4      = [[0 0 0]';vhat4];
S_list  = [S1 S2 S3 S4]
Me       = RpToTrans(I, [L1+L2 0 L1-L4]');
M4       = RpToTrans(I, [L1+L2 0 L1-L4]');
M1       = RpToTrans(I, [0 0 L1]');
M2       = RpToTrans(I, [L1 0 L1]');
M3       = RpToTrans(I, [L1+L2 0 L1]');
M0       = RpToTrans(I, [0 0 0]');
theta1   = deg2rad(0);
theta2   = deg2rad(0);
theta3   = deg2rad(0);
theta4   = 0;
theta_list = [theta1;theta2;theta3;theta4]
dot_theta1 = deg2rad(0); % [deg/sec]
dot_theta2 = deg2rad(0); % [deg/sec]
dot_theta3 = deg2rad(0); % [deg/sec]
dot_theta4 = 0.;         % [m/sec] because joint 4 is a prismatic joint
dot_theta_list = [dot_theta1; dot_theta2;dot_theta3;dot_theta4]
% passing time variables
t0 = 0.0; % [sec]
tend = 10.0; % [sec]
Deltat = 0.25;
N = ceil((tend-t0)/Deltat); % number of interval during the motion where you want to compute the FK

for i=1:N
    t = i*Deltat;
    % example of constant velocities
%     dot_theta1 = deg2rad(10);  % [deg/sec]
%     dot_theta2 = deg2rad(-50); % [deg/sec]
%     dot_theta3 = deg2rad(20);  % [deg/sec]
%     dot_theta4 = -0.03;         % [m/sec] because joint 4 is a prismatic joint
    % example of time-varying joint velocities
    dot_theta1 = 0.2*sin(0.5*t); % [deg/sec]
    dot_theta2 = 0.2*sin(0.6*t); % [deg/sec]
    dot_theta3 = 0.1*sin(1*t); % [deg/sec]
    dot_theta4 = -0.2*sin(0.5*pi*t);         % [m/sec] because joint 4 is a prismatic joint
    dot_theta_list = [dot_theta1; dot_theta2;dot_theta3;dot_theta4]
    theta_list = theta_list + dot_theta_list*Deltat;
    
    Tse = FKinSpace(Me,S_list,theta_list)
    Ts4 = FKinSpace(M4,S_list(:,1:3),theta_list(1:3));
    Ts3 = FKinSpace(M3,S_list(:,1:3),theta_list(1:3));
    Ts2 = FKinSpace(M2,S_list(:,1:2),theta_list(1:2));
    Ts1 = FKinSpace(M1,S_list(:,1),theta_list(1));

    % we can compute the Jacobian by calculating each

    Js = JacobianSpace(S_list, theta_list)
    

    Vs = Js*dot_theta_list
    Tes = TransInv(Tse);
    [Rse,pe] = TransToRp(Tse);
    AdjointTes = Adjoint(Tes);
    Vb = AdjointTes*Vs;
    vb_s = Rse*Vb(4:6)

    figure(1)
    x0=200;     % location of figure in the screen
    y0=200;     % location of figure in the screen
    width=400;  % width of figure
    height=400; % height of figure
    set(gcf,'position',[x0,y0,width,height])
    hold on
    plotvol(L1*2)
    view(116,31)
    xlim([-1 3])
    ylim([-1 1.5])
    zlim([0 1.5])
    plot_sphere([M0(1,4)   M0(2,4)  M0(3,4)], L2/15)
    plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
    plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
    plot_sphere([Ts3(1,4)  Ts3(2,4) Ts3(3,4)], L2/15)
    plot_sphere([Ts4(1,4)  Ts4(2,4) Ts4(3,4)], L2/15)
    plot_sphere([Tse(1,4)  Tse(2,4) Tse(3,4)], L2/15, 'r')
    plot3([M0(1,4) Ts1(1,4)], [M0(2,4) Ts1(2,4)], [M0(3,4) Ts1(3,4)],'k')
    plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k')
    plot3([Ts2(1,4) Ts3(1,4)], [Ts2(2,4) Ts3(2,4)], [Ts2(3,4) Ts3(3,4)],'k')
    plot3([Ts3(1,4) Ts4(1,4)], [Ts3(2,4) Ts4(2,4)], [Ts3(3,4) Ts4(3,4)],'k')
    plot3([Ts4(1,4) Tse(1,4)], [Ts4(2,4) Tse(2,4)], [Ts4(3,4) Tse(3,4)],'k')
    plot_arrow(Tse(1:3,4), (Tse(1:3,4)+vb_s), 'r')
    title(strcat('time = ', num2str(t), ' s'),'interpreter','latex')
    arrow3('update')
    drawnow
    pause(0.1)
    
    % here we save parameters for plotting
    % first the time variable and the enx-effector speed in x,y,z
    time(i) = t;
    xv(i) = vb_s(1);
    yv(i) = vb_s(2);
    zv(i) = vb_s(3);
    % then the joint velocities
    j1(i) = dot_theta1; 
    j2(i) = dot_theta2; 
    j3(i) = dot_theta3; 
    j4(i) = dot_theta4; 

end

figure(2)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
title(strcat('end-effector velocities $\dot{\vec x}_e$, $\dot{\vec y}_e$, $\dot{\vec z}_e$'),'interpreter','latex')
hold on
plot(time,xv)
hold on
plot(time,yv)
plot(time,zv)
xlabel('t [s]')
ylabel('speed [m/s]')
legend('x speed','y speed', 'z speed')

figure(3)
x0=900;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
title('joint-velocities $\dot \theta_1$, $\dot \theta_2$, $\dot \theta_3$, $\dot \theta_4$, $\dot \theta_5$ ','interpreter','latex')
hold on
plot(time,j1)
hold on
plot(time,j2)
plot(time,j3)
plot(time,j4)
xlabel('t [s]')
ylabel('speed [rad/s] or [m/s]')
legend('joint-1 speed [rad/s]','joint-2 speed [rad/s]', 'joint-3 speed [rad/s]', 'joint-4 [m/s]')