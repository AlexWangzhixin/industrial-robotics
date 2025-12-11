% computes the new rotation matrix after a rotation around an arbitrary
% axis

clear all
close all

R0 = eye(3)

omg = [2 2 0.9]

[omg_hat, theta] = AxisAng3(omg)

omg_mat = VecToso3(omg_hat)



R01 = MatrixExp3(omg_mat*theta)




figure(1)
x0=300;     % location of figure in the screen
y0=100;     % location of figure in the screen
width=600;  % width of figure
height=600; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(3)
trplot(R0,'color','b')
hold on
arrow3([0 0 0],omg)
%trplot(R01,'color','r')
disp('----------------- new claculation ----------------------')
Delta_theta = 0.1
theta_new = 0
N = 10;
for i=1:N
    i
    theta_new = theta_new+Delta_theta
    R0_temp = MatrixExp3(omg_mat*theta_new)
    figure(1)
    trplot(R0_temp, 'color', 'r')
    pause(0.1)
    disp('----------------- new claculation ----------------------')
end

if omg(3)>1.2
    disp('correct value')
    R_final = MatrixExp3(omg_mat*theta)
else
        disp('wrong value, try again')
    return
end