% RR_IK_
clear all 
close all
L1 = 1;
L2 = 0.7;

figure(1)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
grid on
volume_length =L1+L2;
plotvol(volume_length)
view([0,90])
link1 = plot([0 0], [0 0], 'k-');
link2 = plot([0 0], [0 0], 'k-');
link3 = plot([0 0], [0 0], 'r-');
link4 = plot([0 0], [0 0], 'r-');
sphere = plot_sphere([0; 0; 0], 0.05,  'r')
xlabel('x [m]')
ylabel('y [m]')
title('point the mouse and click where you want the end-effector')
for i=1:10

% plot stuff
T0 = eye(4); % for plotting purposes only, we define this reference frame at the base

figure(1)
view([0,90])
trplot(T0)
% Get x,y coordinates
[xe, ye] = ginput(1);  
% x = column, not row.  So use yourImage(y,x), not yourImage(x,y)!
%xe = 0.1;
%ye = 0.5;
delete(sphere);
sphere = plot_sphere([xe; ye; 0], 0.05,  'r');
delete(link1);
delete(link2);
delete(link3);
delete(link4);
% solve analytical inverse kinematics
% solution 1: elbow down
L = sqrt(xe^2+ye^2);
gamma = atan2(ye,xe);
alpha = acos((-L2^2+L1^2+L^2)/(2*L1*L));
beta  = acos((-L^2+L1^2+L2^2)/(2*L1*L2));
theta11 = gamma-alpha;
theta21 = pi-beta;

x11 = L1*cos(theta11);
y11 = L1*sin(theta11);
x21 = L1*cos(theta11)+L2*cos(theta21+theta11);
y21 = L1*sin(theta11)+L2*sin(theta21+theta11);

% solution 2: elbow up
theta12 = gamma+alpha;
theta22 = beta-pi;

x12 = L1*cos(theta12);
y12 = L1*sin(theta12);
x22 = L1*cos(theta12)+L2*cos(theta22+theta12);
y22 = L1*sin(theta12)+L2*sin(theta22+theta12);

figure(1)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
view([0,90])
link1 = plot([0 x11], [0 y11], 'k-');
link2 = plot([x11 x21], [y11 y21], 'k-');
link3 = plot([0 x12], [0 y12], 'r-');
link4 = plot([x12 x22], [y12 y22], 'r-');
hold on

disp('--------------------------------------------------------------')
error_x = x21-xe;
error_y = y21-ye;
disp(strcat('the x-position error of the end-effector is: ',' ', num2str(error_x),' [m]'))
disp(strcat('the y-position error of the end-effector is: ',' ', num2str(error_y),' [m]'))
disp('--------------------------------------------------------------')

end
