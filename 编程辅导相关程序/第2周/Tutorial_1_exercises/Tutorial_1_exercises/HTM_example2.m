% - Industrial Robotics Tutorial 1 -
% This exercise creates three rotation matrices and three position vectors
% then computes the associated HTMs and plots them

clear all 
close all
R0 = eye(3);
d0 = [0;0;0];
R01 = rotz(10,'deg')*roty(20,'deg')*rotz(15,'deg');
d01 = [0.2; 0.5; .2];
R02 = rotz(90,'deg')*roty(50,'deg')*rotz(-15,'deg');
d02 = [1.1; 1.2; 1.6];

T0 = RpToTrans(R0, d0);
T01 = RpToTrans(R01, d01);
T02 = RpToTrans(R02, d02);
figure (2)
plotvol(3)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
trplot(T01,'arrow', 'frame', 'b', 'color', 'k')
trplot(T02,'arrow', 'frame', 'c', 'color', 'k')

plot_arrow(T01(1:end-1,4)',T02(1:end-1,4)', 'b--')

plot3([T01(1,4) T02(1,4)], [T01(2,4) T02(2,4)], [T01(3,4) T02(3,4)], 'r', 'Linewidth', 2)
plot3([T0(1,4) T01(1,4)], [T0(2,4) T01(2,4)], [T0(3,4) T01(3,4)], 'r', 'Linewidth', 2)


