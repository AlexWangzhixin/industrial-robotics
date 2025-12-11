% - Industrial Robotics Tutorial 1 -
% This exercise creates three rotation matrices and three position vectors
% then computes the compunding of the three rotation matrices to
% demonstrate how you can get a reference frame in an arbitrary
% configuration by multiplying successive rotation matrices in the current
% frame. Then it creates three HTMs from those rotation matrices and plot
% them. Later (bottom side of the this code), you can do the same exercise
% with the rotations performed wrt fixed frame.
%

clear all 
close all

R0 = eye(3);
d0 = [0;0;0];
R01 = rotz(10,'deg');
d01 = [0; 0; 0];
R02 = rotz(10,'deg')*roty(20,'deg');
d02 = [0; 0; 0];
R03 = rotz(10,'deg')*roty(20,'deg')*rotz(15,'deg');
d03 = [0;0;0];

T0 = RpToTrans(R0, d0);
T01 = RpToTrans(R01, d01);
T02 = RpToTrans(R02, d02);
T03 = RpToTrans(R03, d03);
figure (1)
plotvol(1)
view(129,28)
hold on
trplot(T0,'arrow','frame', '0', 'color', 'k', 'linewidth', 1.5)

figure (2)
plotvol(1)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
trplot(T01,'arrow', 'frame', 'b', 'color', 'b')

figure (3)
plotvol(1)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
%trplot(T01,'arrow', 'frame', 'b', 'color', 'b')
trplot(T02,'arrow', 'frame', 'c', 'color', 'r')

figure (4)
plotvol(1)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
%trplot(T01,'arrow', 'frame', 'b', 'color', 'b')
%trplot(T02,'arrow', 'frame', 'c', 'color', 'r')
trplot(T03,'arrow', 'frame', 'd', 'color', 'g')

% ------------------------------------------------------------------------
% in the case where the compunding was done with respect to the fixed frame
% the multiplication of rotation matrices would be performed in the
% opposite direction and you would achieve a different final reference
% frame
R04 = rotz(15,'deg')*roty(20,'deg')*rotz(10,'deg');
d04 = [0;0;0];
T04 = RpToTrans(R04, d04);
figure (5)
plotvol(1)
view(129,28)
hold on
trplot(T0,'arrow', 'frame', 'a', 'color', 'k')
trplot(T03,'arrow', 'frame', 'd', 'color', 'g')
trplot(T04,'arrow', 'frame', 'd', 'color', 'r')
