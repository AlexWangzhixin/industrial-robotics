% practice exercise with matrix exponential

clear all 
close all

% 1) given a vector of rotation omg that describes a simpe rotation around axis z, 
% and the fixed reference frame orientation R0, 
% compute the new orientation of the reference frame R01 after rotation around omg
omg = [0 0 2]
[omg_hat, theta] = AxisAng3(omg)

omg_skew = VecToso3(omg_hat)

R0 = eye(3)
R01 = MatrixExp3(omg_skew*theta)

figure(1)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=300; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(2)
trplot(R0)
hold on
arrow3([0 0 0], omg_hat)
hold on
trplot(R01, 'color','r')
% --------------------------------------------------------------------------

% 2) given an arbitrary vector of rotation omg, and the fixed reference frame orientation
% R0, compute the new orientation of the reference frame R01 after rotation
% around omg
omg = [1 1 2]
[omg_hat, theta] = AxisAng3(omg)

omg_skew = VecToso3(omg_hat)

R0 = eye(3)
R01 = MatrixExp3(omg_skew*theta)

figure(2)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=300; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(2)
trplot(R0)
hold on
arrow3([0 0 0], omg_hat)
hold on
trplot(R01, 'color','r')

% 3) given the vector of rotation omg of exercise (2), show the action of rotation
% of the reference frame R01 after rotation around omg
omg = [1 1 2]
[omg_hat, theta] = AxisAng3(omg)

omg_skew = VecToso3(omg_hat)

R0 = eye(3)
N = 10;
theta_new = 0;
figure(3)
x0=400;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=800;  % width of figure
height=600; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(2)
view(104,43)
trplot(R0)
hold on
arrow3([0 0 0], omg_hat)
    
for i=1:N-1
    theta_new =theta_new + theta/(N-1)
    R01 = MatrixExp3(omg_skew*theta_new);
    
    figure(3)
    trplot(R01, 'color','r')
    pause(0.4)
end
R01 = MatrixExp3(omg_skew*theta)
figure(3)
trplot(R01, 'color','k')
