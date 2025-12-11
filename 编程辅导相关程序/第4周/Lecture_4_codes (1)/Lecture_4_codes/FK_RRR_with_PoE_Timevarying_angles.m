% FK example of an RRR serial spatial manipulator solved with PoE where you
% specify a joint speed rotation and solve for the forward kinematic in
% time

clear all
close all
format short

%define geometrical parameters
theta1=deg2rad(0);
theta2=deg2rad(0);
theta3=deg2rad(0);
L1 = 2;
L2 = 1;
%define joint rotational speed
theta1dot = deg2rad(15); % [deg/sec] rotational speed of joint 1
theta2dot = deg2rad(4); % [deg/sec] rotational speed of joint 2
theta3dot = deg2rad(7); % [deg/sec] rotational speed of joint 3

%define base frame
T_s = eye(4);
%define end-effector frame wrt {s}
M = [0 0 1 L1;  ...
     0 1 0 0;    ...
    -1 0 0 -L2;  ...
     0 0 0 1];
%for plotting purposes only, we can define the home config of joint-2
%Notice that we don't need this to only compute the end-effectore pose
R2 = rotx(pi/2)*rotz(-pi/2); % this command set the orientation of joint-2 with the z axis aligned with the rotation axis
d2 = [L1 0 0]';
M2 = RpToTrans(R2,d2); % this defines the HTM of joint-2 in the home configuation

% plot the manipulator at its home configuration
volume_length = 4;
figure(1)
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(volume_length)
view(37,34)
%trplot(T_s)
hold on
plot3([0 T_s(1,4)],[0 T_s(2,4)],[-volume_length T_s(2,4)], 'o-k','linewidth',1)
plot3([T_s(1,4) M2(1,4)],[T_s(2,4) M2(2,4)],[T_s(3,4) M2(3,4)], 'o-k','linewidth',1)
plot3([M2(1,4) M(1,4)],[M2(2,4) M(2,4)],[M2(3,4) M(3,4)], 'o-k','linewidth',1)
% trplot(T_s,'length', 0.2,'color','k')
% trplot(M2,'length', 0.2,'color','k')
% trplot(M,'length', 0.2,'color','k')


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

% compute the FK in time
deltaT = 0.05; % [s] timestep 
finalT = 15.0; % [s] total duration of the simulatin
nstep  = floor(finalT/deltaT) % number of timesteps over which I solve FK

for i=1:nstep
    timestep(i)=i;
    % the following command assigns a progressive increasing value of the
    % joint angles:
    theta1dot = 3*cos(i*deltaT);
    theta2dot = 2.5*cos(3.5*i*deltaT);

    theta1=theta1+theta1dot*deltaT;
    theta2=theta2+theta2dot*deltaT;
    theta3=theta3+theta3dot*deltaT;
    
    % solve the FK by multiplying the matrix exponential of the screw axes
    T_s1 = MatrixExp6(S1mat*theta1)*T_s;
    T_s2 = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*M2;
    tic;
    T_se = MatrixExp6(S1mat*theta1)*MatrixExp6(S2mat*theta2)*MatrixExp6(S3mat*theta3)*M;
    time_elapsed(i) = toc;
    % plot some stuff
    figure(1)
    link3 = plot3([T_s1(1,4) T_s2(1,4)],[T_s1(2,4) T_s2(2,4)],[T_s1(3,4) T_s2(3,4)], 'o-r','linewidth',1);
    link4 = plot3([T_s2(1,4) T_se(1,4)],[T_s2(2,4) T_se(2,4)],[T_s2(3,4) T_se(3,4)], 'o-r','linewidth',1);
    plot3(T_se(1,4),T_se(2,4),T_se(3,4), '.r')
    drawnow
    pause(0.05)
    delete(link3);
    delete(link4);
    % below I save some data which I can use for plotting later
    EEx(i) = T_se(1,4); % save the x-position of the end-effector at each timestep
    EEy(i) = T_se(2,4); % save the y-position of the end-effector at each timestep
    EEz(i) = T_se(3,4); % save the z-position of the end-effector at each timestep
    ja1(i) = rad2deg(theta1); % save the value of joint1 at each timestep
    ja2(i) = rad2deg(theta2); % save the value of joint2 at each timestep
    ja3(i) = rad2deg(theta3); % save the value of joint3 at each timestep
end
% plot the final configuration of the manipulator
figure(1)
hold on
link1 = plot3([0 T_s(1,4)],[0 T_s(2,4)],[-volume_length T_s(2,4)], 'o-r','linewidth',1);
link2 = plot3([T_s(1,4) T_s1(1,4)],[T_s(2,4) T_s1(2,4)],[T_s(3,4) T_s1(3,4)], 'o-r','linewidth',1);
link3 = plot3([T_s1(1,4) T_s2(1,4)],[T_s1(2,4) T_s2(2,4)],[T_s1(3,4) T_s2(3,4)], 'o-r','linewidth',1);
link4 = plot3([T_s2(1,4) T_se(1,4)],[T_s2(2,4) T_se(2,4)],[T_s2(3,4) T_se(3,4)], 'o-r','linewidth',1);
frame3 = trplot(T_se,'length', 0.2,'color','b');

figure(2)
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plot(timestep*deltaT, EEx, 'k')
hold on
plot(timestep*deltaT, EEy, 'b')
plot(timestep*deltaT, EEz, 'r')
xlabel('time [s]')
ylabel('position [m]')
legend('x position','y position', 'z position')
title('position of the end effector in time')



figure(4)
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plot(timestep*deltaT, ja1, 'k')
hold on
plot(timestep*deltaT, ja2, 'b')
plot(timestep*deltaT, ja3, 'r')
xlabel('time')
ylabel('joint angle [rad]')
legend('joint-1','joint-2', 'joint-3')
title('joint angle values in time')