% example of the usage of the matrix logarithm.
% if we are given a frame {b} that is the result of an unknown rotation, we
% can compute the axis of rotation omega_hat and the amount of rotation
% theta around that axis which transforms {s} into {b}
clear all
close all
format short

%generate the fixed frame frame
T_s = eye(4);
%generate an arbitrary frame {b} rotated wrt {s}
T_sb = RpToTrans((rotz(30,'deg')*roty(-20,'deg')),[0;0;0]);

% to compute the matrix logarithm, we need to first extract the rotation
% matrix out of the HTM:
[R_sb,p]=TransToRp(T_sb)
% then we compute the matrix logarithm which gives omega in skew symmetric
% form
omega_mat = MatrixLog3(R_sb)
%then we want the unit axis of rotation and the angle, so we first transform omega in vector form:
omega = so3ToVec(omega_mat)
% now we can compute the unit axis of rotation and the angle around it
[omega_hat, theta] = AxisAng3(omega) 

figure(1)
x0=400;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=800;  % width of figure
height=600; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(1)
hold on
hold on
trplot(T_s,'frame','s','arrow','color','k')
trplot(T_sb,'frame','b', 'arrow','color','b')
plot_arrow([0;0;0], omega_hat, 'r')
