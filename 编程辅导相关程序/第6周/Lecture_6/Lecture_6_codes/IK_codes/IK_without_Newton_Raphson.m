% this code tries to solve the IK of a RR planar manipulator in an
% iterative manner, but without using the Newton Raphson algorithm. It
% simply loops over the FK by testing for random values of the joint
% coordinates until the required tolerance between the FK solution and the
% desired position of the end-effector is met

clear all
close all
format short

%RRR jacobian and inverse kinematics
syms theta1 theta2 theta3 L1 L2 real
theta1=deg2rad(0);
theta2=deg2rad(0);
L1 = 1;
L2 = 1;

Ts =  eye(4);
M3 = RpToTrans(eye(3), [L1+L2 0 0]');
M2 = RpToTrans(eye(3), [L1 0 0]');
M1 = RpToTrans(eye(3), [0 0 0]');

omghat1 = [0 0 1]';
omghat2 = [0 0 1]';
q1 = [0 0 0]';
q2 = [L1 0 0]';
vhat1 = -cross(omghat1, q1);
vhat2 = -cross(omghat2, q2);
S1 = [omghat1;vhat1];
S2 = [omghat2;vhat2];
S1mat = VecTose3(S1);
S2mat = VecTose3(S2);
% FK
theta1=deg2rad(10);
theta2=deg2rad(50);
Ts3 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*M3;
Ts2 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*M2;
Ts1 = MatrixExp6(S1mat*theta1)*M1;

%plot stuff
volume_length =2;
figure(1)
grid on
plotvol([-volume_length volume_length -1 2])
view([0,90])
trplot(Ts)
hold on
plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'o-k','linewidth',1)
plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'o-k','linewidth',1)


% compute jacobians
% first compute geometric jacobian for the twist
Js1 = S1;
Js2 = Adjoint(Ts1)*S2;
J = [Js1 Js2]
% then compute analytic jacobian from differentiation of the forward
% kinematic solution
Janal = [-L1*sin(theta1)-L2*sin(theta1+theta2) -L2*sin(theta1+theta2);...
     L1*cos(theta1)+L2*cos(theta1+theta2) L2*cos(theta1+theta2); ...
     0 0;...
     0 0;...
     0 0;...
     1 1]
% assign values of joint velocities
dtheta1 = 3.14/10; % rad/sec
dtheta2 = 6.28/10; % rad/sec
dtheta  = [dtheta1 dtheta2]'; 
Re = TransToRp(Ts3);
% compute the twist in space of the end effector
Vs = J*dtheta;
%mod_omg = norm(Vs(1:3));
%Ss = Vs/mod_omg;
%q = to compute the position of the screw you must do the inverse of the
%cross product (we did it somehwere in previous codes)
% compute inverse adjoint of Ts3
AdT3s = Adjoint(TransInv(Ts3));
% compute body twist
Vb = AdT3s*Vs
% compute body jacobian
Jb1 = AdT3s*Js1;
Jb2 = AdT3s*Js2;
dot_xe_fromTwist =  Re*Vb(4:6)
% compute end-effector velocity in generalized coordinates
x_e = Janal*dtheta
plot_arrow(Ts3(1:3,4),Ts3(1:3,4)+Re*Vb(4:6),'b--', 0.5)
plot_arrow(Ts3(1:3,4),Ts3(1:3,4)+Re*Jb1(4:6), 'k-.',0.5)
plot_arrow(Ts3(1:3,4),Ts3(1:3,4)+Re*Jb2(4:6), 'r-.',0.5)


xpos = -1.2;
ypos = 0.7;
xd = [xpos  ypos]';
theta1_guess = deg2rad(0); 
theta2_guess = deg2rad(0);
theta_guess = [theta1_guess; theta2_guess]

figure(2)
grid on
plotvol([-volume_length volume_length+0.5 -1.5 1.5])
hold on
view([0,90])
trplot(Ts)
plot_sphere([xpos; ypos; 0], 0.05,  'r');

e = [100 100]';
tolerance = 0.1;
h = 0;
while norm(e)>tolerance
    theta_guess = rad2deg([rand*6.28 rand*6.28]')
    h = h+1;
    Ts3 = MatrixExp6(S1mat*theta_guess(1))*MatrixExp6(S2mat*theta_guess(2))*M3;
    Ts2 = MatrixExp6(S1mat*theta_guess(1))*MatrixExp6(S2mat*theta_guess(2))*M2;
    Ts1 = MatrixExp6(S1mat*theta_guess(1))*M1;
    xe = Ts3(1:2,4);
    e = (xd - xe);
    norm(e);
    fprintf(strcat('attempt number: ', num2str(h), '\n'))
    fprintf(strcat('error: ', num2str(norm(e)), '[m] \n'))
    fprintf('---------------------------------------------------------\n')
    
    figure(2)
    link1 = plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'o-.k','linewidth',0.5);
    hold on
    link2 = plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'o-.k','linewidth',0.5);
    plot(Ts3(1,4), Ts3(2,4), 'b.')
    hold on  
end

figure(2)
link1 = plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'o-k','linewidth',2);
hold on
link2 = plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'o-k','linewidth',2);
plot(Ts3(1,4), Ts3(2,4), 'r.')
hold on
