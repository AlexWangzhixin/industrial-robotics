% This code allows you to test the behaviour of a HTM subject to the spiral motion of a Screw
% This code is intended for you to understand the effect of changing the characteristics of the Screw
% by changing q, h and omega:
clear all
close all
% q = [0 0 0];            % vector which defines distance between frame and screw axis
% h = 0.;                 % pitch of the screw defines how much frame moves along the screw for each rotation
% omega = [0 0 2*pi];     % rotation vector omega_hat*theta, defines the axis of rotation and amount of rotation
%-------------------------
q = [2 0 0];
h = 0.;
omega = [0 0 2*pi];
%-------------------------
% q = [2 0 0];
% h = 0.2;
% omega = [0 0 2*pi];
%-------------------------
% q = [1.5 0 0];
% h = 0.2;
% omega = [6 6 6];
%------------------------
% q = [0 0 0];
% h = 0.2;
% omega = [0 0 2*pi];
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
    S= [omega_hat'; (-cross(omega_hat,q)+h*omega_hat)'];    % computes the Screw Axis in vector form
    S_mat = VecTose3(S);                                    % computes the skew-symmetric form of the screw axis
    theta_increment = i*theta/(n);                          % computes the amount of rotation of theta at step "i"
    T1 = MatrixExp6(S_mat*(theta_increment));               % computes HTM through matrix exponential of the Screw Axis
    %---------------------   plotting  -----------------------------------
    % the following command plot the figure again at each iteration of "i"
    
%     figure(1)
%     hold on
%     plotvol(4)
%     view(55,35)
%     trplot(T0)
%     plot_arrow(q,omega_hat+q, 'r')
    %trplot(T1, 'color', 'r')
    x1 =T1(1,4);
    y1 = T1(2,4);
    z1 = T1(3,4);
    plot3(x1,y1,z1,'.','color','k')
    pause(0.1)
    %hold off
end
trplot(T1, 'color', 'r')