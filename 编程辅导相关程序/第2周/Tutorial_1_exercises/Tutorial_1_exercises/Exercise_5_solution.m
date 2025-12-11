% Industrial Robotics - tutorial 1
% Given a point P and two transformations involving: a rotation and a
% translation, compute and plot the resulting HTMs and calculate the
% coordinate of point P as seen from the three HTMs. Finally plot a vector
% that connects the last HTM to P.

clear all
close all


Ts = eye(4);
figure(1)
plotvol(4)
hold on
view(136,43)
trplot(Ts,'arrow','frame', 's','color','k')

ps = [1 1 0]'

figure(1)
plot_sphere(ps,0.05,'r') % this plots a point in space in location ps, with radius 0.05 and color 'red'

% the following command describe the first transformation
Rsb = rotz(pi/2);
dsb = [0 0 0]';
Tsb = RpToTrans(Rsb, dsb);
figure(1)
trplot(Tsb,'arrow','frame', 'b','color','r')
Rbs = RotInv(Rsb)
pb = Rbs*ps

% -----------------------------------------------------------------
%solve the same problem in the case where the next transpormation did 
%displace the HTM

Rbc = eye(3);% this command implies that the new transformation does not rotate
dbc = [0 2 0]';% this command says that the new transformation displaces Tsb of 2 unit lengths along y_b
Tbc = RpToTrans(Rbc,dbc);% this creates the transformation Tbc

%if I want to plot the position and orientation of reference frame {c} I
%need to express it as if it was seen from {c}, therefore I need to compute
%Tsc:
Tsc = Ts*Tsb*Tbc;
trplot(Tsc, 'arrow','frame', 'c','color','b')
%now I want to compute p from {c}. To do that I need to solve pc = Tcs*ps,
%however, I have only Tsc, therefore I need to invert it to get Tcs. To do
%that I use the function TransInv()
Tcs = TransInv(Tsc)
%now I can solve the equation pc = Tcs*ps. However, notice that ps has 3
%rows, while Tcs has 4 rows. Hence I need to express ps in the "homogenous
%form" by adding a 1 in the forth row, this can be done by doing [ps',1]'
pc = Tcs*[ps',1]'
% to plot a vector that goes from the origin of {c} to P, I need to use the
% function plot_arrow and feed it with the coordinates of the origin of {c}
% this are simply represented by the position vector of Tsc, i.e. the 3
% elements in the forth column: Tsc(1:3,4)
plot_arrow(Tsc(1:3,4),ps')
