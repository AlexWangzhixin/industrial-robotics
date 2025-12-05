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
theta1   = deg2rad(40);
theta2   = deg2rad(10);
theta3   = deg2rad(-20);
theta4   = 0;
theta_list = [theta1;theta2;theta3;theta4]

Tse = FKinSpace(Me,S_list,theta_list)
Ts4 = FKinSpace(M4,S_list(:,1:4),theta_list(1:4));
Ts3 = FKinSpace(M3,S_list(:,1:3),theta_list(1:3));
Ts2 = FKinSpace(M2,S_list(:,1:2),theta_list(1:2));
Ts1 = FKinSpace(M1,S_list(:,1),theta_list(1));

% we can compute the Jacobian by calculating each

Js = JacobianSpace(S_list, theta_list)
dot_theta1 = deg2rad(10); % [deg/sec]
dot_theta2 = deg2rad(20); % [deg/sec]
dot_theta3 = deg2rad(-10); % [deg/sec]
dot_theta4 = -0.3;         % [m/sec] because joint 4 is a prismatic joint

dot_theta_list = [dot_theta1; dot_theta2;dot_theta3;dot_theta4]
Vs = Js*dot_theta_list
Tes = TransInv(Tse);
[Rse,pe] = TransToRp(Tse);
AdjointTes = Adjoint(Tes);
Vb = AdjointTes*Vs;
vb_s = Rse*Vb(4:6)

figure(1)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(L1*3)
view(15,26)
xlim([-0.2 3])
ylim([-0.2 1.5])
zlim([0 1.5])
plot_sphere([M0(1,4)   M0(2,4)  M0(3,4)], L2/15)
plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
plot_sphere([Ts3(1,4)  Ts3(2,4) Ts3(3,4)], L2/15)
plot_sphere([Ts4(1,4)  Ts4(2,4) Ts4(3,4)], L2/15)
plot_sphere([Tse(1,4)  Tse(2,4) Tse(3,4)], L2/15)
plot3([M0(1,4) Ts1(1,4)], [M0(2,4) Ts1(2,4)], [M0(3,4) Ts1(3,4)],'k')
plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k')
plot3([Ts2(1,4) Ts3(1,4)], [Ts2(2,4) Ts3(2,4)], [Ts2(3,4) Ts3(3,4)],'k')
plot3([Ts3(1,4) Ts4(1,4)], [Ts3(2,4) Ts4(2,4)], [Ts3(3,4) Ts4(3,4)],'k')
plot3([Ts4(1,4) Tse(1,4)], [Ts4(2,4) Tse(2,4)], [Ts4(3,4) Tse(3,4)],'k')
plot_arrow(Tse(1:3,4), (Tse(1:3,4)+vb_s), 'r')
arrow3('update')
