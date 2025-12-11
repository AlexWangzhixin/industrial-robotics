% exercises from Industrial Robotics number 1
% this exercise creates a reference frame for the base frame and another
% for showing how to create HTMs and plotting them

clear all %this command deletes all variable and parameters previously created
close all %this command closes all figures and files previously opened

% the following command creates a rotation matrix that represents the fixed
% frame, therefore it is defined by the identity matrix I
Rs = [1 0 0;
      0 1 0;
      0 0 1]

% the following command creates a displacement vector for the fixed
% reference frame. Because the fixed reference frame, by definition, is the
% origin, then its displacement is zero in all directions.
% notice that the apex symbol "]'" means transpose, therefore this command creates a 3x1 vertical vector
% if you did not use the ' symbol, it would create a 1x3 horizontal vector
ds = [0 0 0]'

% the following command uses the function "RpToTrans()" to create an HTM
% from a rotation matrix and a displacement vector
Ts = RpToTrans(Rs, ds)

% the following command creates a rotation matrix using the elementary
% rotation around x. Here I specify that the angle of rotation is expressed
% in degrees "deg". If I didn't specify it, Matlab would assume that you
% are requesting the angle in radians
Rs1 = rotx(20, 'deg')
ds1 = [0 1 0]'
Ts1 = RpToTrans(Rs1, ds1)

% the commands below are only taking care of the plotting of the two HTMs
% created before:
figure(1) % this command makes a generic figure with nothing inside
plotvol(3)% this command says how big is the volume where we plot stuff
view(-145, 48)% this commands sets the perspective from which the figure is shown
trplot(Ts) % this command plots the HTM named "Ts"
hold on % this command makes sure that the things you have plotted before remain in the figure (they are not over-written)
trplot(Ts1, 'color', 'r') % this command plots a second HTM called "Ts1" and colors it red

