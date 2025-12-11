% an example of the usage of angle and axis screw elements
% if we have a frame {s} and a unit axis omega, we can immediately
% compute how {s} will look like after having rotated around omega for an amount theta 
clear all
close all
format short 

T_s = eye(4);
omega = [1;3;1];
% "manual" approach to computing the unit axis of rotationa and the angle
theta = norm(omega)  
omega_hat = omega/norm(omega)
% alternatively we could use this command:
[omega_hat, theta]=AxisAng3(omega)

% first we compute the skew symmetric form of the unit axis omega
omega_mat = VecToso3(omega_hat)
% then we compute the matrix exponential of omega_mat and theta
exponential_coord = omega_mat*theta
T_b = MatrixExp3(exponential_coord)

figure(1)
plotvol(max(omega))
hold on
view(116,40)
trplot(T_s,'frame','s','arrow','color','k')
pause(0.5)
plot_arrow([0;0;0], omega_hat, 'r')
pause(0.5)
trplot(T_b,'frame','b', 'arrow','color','b')
