% This code allows you to test the behaviour of a HTM subject to a
% prismatic joint. A Screw axis representation of a revolute joint has the
% rotational part of the screw equal to zero, and only has translation
% along the Screw axis
clear all
close all
%h=0.3;                 
% q = [0 0 0];          % vector which defines distance between frame and screw axis
% omega = [1 pi 2*pi];     % rotation vector omega_hat*theta, defines the axis of rotation and amount of rotation
%-------------------------
q = [2 0 0];
h = 0.5;
omega = [0 0 2*pi];
%------------------------
[omega_hat, theta] =AxisAng3(omega) % decomposes rotation vector in unit-axis of rotation and angle of rotation
T0 = eye(4);        % defines initial reference frame

%--------------------------------------------------------------------------
% following commands only set up some details for the figure
figure(1)   
x0=300;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=800;  % width of figure
height=800; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
hold on
plotvol(4)  % defines the size of the xyz volume we want to plot
view(55,35) % defines the angle of the camera to look at the figure
trplot(T0)  % plots T0
plot_arrow(q,omega_hat+q, 'r')  % plots the screw axis unit vector
%--------------------------------------------------------------------------

n = 40;     % defines the number of sequential steps we want to plot during the rotation
for i=1:n
    %---------------------   calculations   ------------------------------
    S= [[0 0 0]'; (h*omega_hat)'];    % computes the Screw Axis in vector form
    S_mat = VecTose3(S);                                    % computes the skew-symmetric form of the screw axis
    theta_increment = i*theta/(n);                          % computes the amount of rotation of theta at step "i"
    T1 = MatrixExp6(S_mat*(theta_increment));               % computes HTM through matrix exponential of the Screw Axis
    %---------------------   plotting  -----------------------------------
    % the following command plot the figure again at each iteration of "i"
    % comment line 40 to 45 to show whole motion evolution
    figure(1)
    hold on
    plotvol(4)
    view(55,35)
    trplot(T0)
    plot_arrow(q,omega_hat+q, 'r')
    plot3([q(1)+T1(1,4) T1(1,4)],[q(2)+T1(2,4) T1(2,4)],[q(3)+T1(3,4) T1(3,4)],'color','r','linewidth', 1)
    trplot(T1, 'color', 'r')
    pause(0.05)
    %hold off
end
