% this script creates a point-to-point trajectory by defining the final and
% initial configuration of the end-effector and calling the
% CartesianTrajectory function which computes N configuration of the
% manipulator between the final and initial configuration based on a cubic
% time-scaling.
clear all; 
close all
format short;
figure (1)
% create volume for eithe a 3d case or a planar 2d case, comment out
% accordingly
%plotvol([-(0.5) (2) -(0.5) (1.5) -1 6 ]);
plotvol([-1 3 -1 3 ])
%view(28,37)
% initial configuration of the end effector expressed as a HTM either 3d or
% 2d, comment out accordingly
Xstart = [[1, 0, 0, 1]; [0, 1, 0, 0]; [0, 0, 1, 1]; [0, 0, 0, 1]];
%Xstart = RpToTrans(eye(3), [0 0 0]');
% final configuration of the end effector
Xend = [[0, 0, 1, 0.1]; [1, 0, 0, 0]; [0, 1, 0, 4.1]; [0, 0, 0, 1]];
%Xend = RpToTrans(rotz(pi/2), [2 1 0]')
% duration of the motion from Xstart to Xend
Tf = 5;
% number of intermediate configurations of the end-effector between Xstart
% and Xend
N = 30;
% method of time-scaling interpolation: 3 stands for cubic and 5 stands for
% quintic interpolation
method = 3;
% call the function which generates the point-to-point trajectory
traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
% extract from the data strucure "traj" the N HTMs 
Ts = zeros(4,4,N)
for i=1:N
    Ts(:,:,i)=cell2mat(traj(i));
    figure (1)
    trplot(Ts(:,:,i), 'length', 0.3)
    hold on
    plot_sphere(Ts(1:3,4,i), 0.01)
    hold on
    x(i) = Ts(1,4,i);
    y(i) = Ts(2,4,i);
    z(i) = Ts(3,4,i);

end

figure(2) 
plot(x)
grid on
hold on
plot(y)
plot(z)
legend('$x$','$y$','$z$', 'fontsize',13, 'interpreter', 'latex','location','northwest')
xlabel('time [sec]','fontsize',13,'interpreter', 'latex')
ylabel('position [m]','fontsize',13,'interpreter', 'latex')
title('trajectory cubic time scaling','fontsize',13,'interpreter', 'latex')


% traj_2 = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% % extract from the data strucure "traj" the N HTMs 
% Ts = zeros(4,4,N)
% for i=1:N
%     Ts(:,:,i)=cell2mat(traj_2(i));
%     figure (1)
%     trplot(Ts(:,:,i), 'color','r','length', 0.3)
%     hold on
%     plot_sphere(Ts(1:3,4,i), 0.01, 'r')
% 
% end

