% this code is an example of how to extrac the XYZ euler angles from an
% arbitrary rotation matrix
clear all 
close all
R0 = eye(3);
d = [0 0 0]';
R1 = R0*rotx(10,'deg');
R2 = roty(20,'deg')*R1;
R3 = rotz(30,'deg')*R2;

figure(1)
view([116,33])
trplot(RpToTrans(R0,d), 'color', 'k')

figure(2)
view([116,33])
trplot(RpToTrans(R0,d), 'color', 'k')
hold on
trplot(RpToTrans(R1,d))

figure(3)
view([116,33])
trplot(RpToTrans(R0,d), 'color', 'k')
hold on
trplot(RpToTrans(R2,d))

figure(4)
view([116,33])
trplot(RpToTrans(R0,d), 'color', 'k')
hold on
trplot(RpToTrans(R3,d))

ZYX = rad2deg(rotm2eul(R3, 'ZYX'))
