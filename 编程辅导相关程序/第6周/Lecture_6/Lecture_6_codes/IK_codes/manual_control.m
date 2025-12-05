% mimic manual control of a RR planar manipualtor

clear all
close all

% set length of links 1 and 2
L1 = 0.5;
L2 = 0.5;


% plot stuff
figure(1)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
T0 = eye(4); % for plotting purposes only, we define this reference frame at the base
grid on
volume_length =L1+L2+0.5;
plotvol(volume_length)
view([0,90])
xlabel('x [m]')
ylabel('y [m]')
hold on
trplot(T0)
% define desired position of the end-effector
xe = 0.2;
ye = -0.5;
plot_sphere([xe; ye; 0], 0.05,  'r');


% definite initial state of the manipulator
theta11 = 0;
theta21 = 0;
x11 = L1*cos(theta11);
y11 = L1*sin(theta11);
x21 = L1*cos(theta11)+L2*cos(theta21+theta11);
y21 = L1*sin(theta11)+L2*sin(theta21+theta11);

figure(1)
x0=600;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(volume_length)
hold on
view([0,90])
xlabel('x [m]')
ylabel('y [m]')
trplot(T0)
plot_sphere([xe; ye; 0], 0.05,  'r');
title('press up/down for joint-1 and left/right for joint-2')
link1 = plot([0 x11], [0 y11], 'k-');
link2 = plot([x11 x21], [y11 y21], 'k-');
hold on

k=0;
while k ~=13
k = waitforbuttonpress;
% 28 leftarrow
% 29 rightarrow
% 30 uparrow
% 31 downarrow
value = double(get(gcf,'CurrentCharacter'));
if value == 30
    delta_theta11 = deg2rad(5);
    theta11 = theta11+delta_theta11;
    disp('rotate jont-1 5 degrees')
elseif value == 31
    delta_theta11 = -deg2rad(5);
    theta11 = theta11+delta_theta11;
    disp('rotate jont-1 -5 degrees')    
elseif value == 28
    delta_theta21 = deg2rad(5);
    theta21 = theta21+delta_theta21;
    disp('rotate jont-2 5 degrees')
elseif value == 29
    delta_theta21 = -deg2rad(5);
    theta21 = theta21+delta_theta21;
    disp('rotate jont-2 -5 degrees')
else
    return
end
    % compute forward kinematic to estimate end-effector position
    x11 = L1*cos(theta11);
    y11 = L1*sin(theta11);
    x21 = L1*cos(theta11)+L2*cos(theta21+theta11);
    y21 = L1*sin(theta11)+L2*sin(theta21+theta11);

    figure(1)
    x0=600;     % location of figure in the screen
    y0=200;     % location of figure in the screen
    width=400;  % width of figure
    height=400; % height of figure
    set(gcf,'position',[x0,y0,width,height])
    plotvol(volume_length)
    hold on
    view([0,90])
    xlabel('x [m]')
    ylabel('y [m]')
    trplot(T0)
    plot_sphere([xe; ye; 0], 0.05,  'r');
    title('press up/down for joint-1 and left/right for joint-2')
    link1 = plot([0 x11], [0 y11], 'k-');
    link2 = plot([x11 x21], [y11 y21], 'k-');
    drawnow
    
    % compute error betwee current position and desired position of
    % end-effector
    error_x = x21-xe;
    error_y = y21-ye;
    disp(strcat('the x-position error of the end-effector is: ',' ', num2str(error_x),' [m]'))
    disp(strcat('the y-position error of the end-effector is: ',' ', num2str(error_y),' [m]'))
    disp('--------------------------------------------------------------')
end

