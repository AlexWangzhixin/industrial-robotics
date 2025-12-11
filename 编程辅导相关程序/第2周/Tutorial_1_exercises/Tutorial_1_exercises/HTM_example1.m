% - Industrial Robotics Tutorial 1 -
% This exercise creates three rotation matrices and three position vectors
% then creates the corrisponding HTMs and computes the compunding of these
% to demonstrate how you can get a reference frame in an arbitrary
% configuration by multiplying successive HTMs in the current
% frame. 

clear all 
close all

R0 = eye(3);
d0 = [0;0;0];
T0 = RpToTrans(R0, d0);

R01 = rotz(10,'deg');
d01 = [1; 0; 0];
T01 = RpToTrans(R01, d01);

R12 = roty(20,'deg');
d12 = [1; 1; 0];
T12 = RpToTrans(R12, d12);


R23 = rotz(15,'deg');
d23 = [1;1;1];
T23 = RpToTrans(R23, d23);

T01 = T0*T01;
T02 = T0*T01*T12;
T03 = T0*T01*T12*T23;

figure (1)
plotvol(4)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')

figure (2)
plotvol(4)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
trplot(T01,'arrow', 'frame', 'b', 'color', 'b')

figure (3)
plotvol(4)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
%trplot(T01,'arrow', 'frame', 'b', 'color', 'b')
trplot(T02,'arrow', 'frame', 'c', 'color', 'r')

figure (4)
plotvol(4)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
%trplot(T01,'arrow', 'frame', 'b', 'color', 'b')
%trplot(T02,'arrow', 'frame', 'c', 'color', 'r')
trplot(T03,'arrow', 'frame', 'd', 'color', 'g')


