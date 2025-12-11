% example of a 2R serial planar manipulator and how to loop over the joint
% angles to draw the Workspace. Follow the commented lines in the code to
% test various configuration of actuation.

clear all
close all
format short

theta1 = deg2rad(30);
theta2 = deg2rad(0);
L1 = 0.5;
L2 = 0.4;

for i = 1:5000
    i;
    % if you want to draw the workspace by randomly iterating over all
    % possible value of theta1 and theta2 between 0 and 360
    %theta1 = deg2rad(rand*360);
    %theta2 = deg2rad(rand*360);
    % if you want to impose constraint on the manipulator, say the joint
    % can only rotate 90 degrees
    theta1 = deg2rad(rand*360);
    theta2 = deg2rad(rand*360); 
    % if your joints have no constraints and can span 180deg 
    %theta1 = deg2rad(rand*180);
    %theta2 = deg2rad(rand*180); 
    %---------------------------------------------------------------------
    % calulation of rotation matrices, displacement vectors and HTMs
    Rs0 = rotz(theta1);
    ds0 = [0;0;0];
    R01 = rotz(theta2);
    d01 = [L1;0; 0];
    R12 = eye(3);
    d12 = [L2; 0; 0];
    Ts0 = RpToTrans(Rs0,ds0);
    T01 = RpToTrans(R01, d01);
    T12 = RpToTrans(R12, d12);
    Ts1 = Ts0*T01;
    Ts2 = Ts1*T12;
    % the follwoing command saves the x y and angle of the joints so that
    % they can be plotted later alotgether
    x_ee(i) = Ts2(1,4);
    y_ee(i) = Ts2(2,4);
    jc1(i) = rad2deg(theta1);
    jc2(i) = rad2deg(theta2);

end

% this figure plots the relationship between joint angles and end-effector
% position in a 3d plot
figure(1)
x0=200;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=600;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
scatter3(jc1,jc2,x_ee,'.')
hold on
scatter3(jc1,jc2,y_ee,'r.') 
xlabel('joint coordinate 1')
ylabel('joint coordinate 2')
zlabel('position of the end effector')
legend('x position','y position')

% this figure plots the workspace by adding a point to each position
% covered by the end-effector
figure(2)
view(0,90)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
hold on
plot(x_ee,y_ee,'.')
trplot(eye(4), 'length', L2/10)
plot3([0 Ts1(1,4)], [0 Ts1(2,4)], [0 Ts1(3,4)], 'k','Linewidth', 2)
plot3([Ts1(1,4) Ts2(1,4)], [Ts1(2,4) Ts2(2,4)], [Ts1(3,4) Ts2(3,4)],'k','Linewidth', 2)
plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/30)
plot_sphere([Ts1(1,4)  Ts1(2,4) Ts1(3,4)], L2/15)
plot_sphere([Ts2(1,4)  Ts2(2,4) Ts2(3,4)], L2/15)
axis equal 
xlabel('x position')
ylabel('y position')
title('Workspace of a RR planar serial manipulator')