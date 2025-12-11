% - Industrial Robotics Tutorial 1 -
% solution to exercise 6 of the tutorial

clear all % this command closes all previous dataset and figures
close all % this command closes files which might have been opened before

T_a = eye(4) % this command creaes a HTM called T_a = I

T_db = [ 0  0 -1  250; 
         0 -1  0 -150;
        -1  0  0  200;
         0  0  0    1]
     
T_de = [ 0  0 -1  300; 
         0 -1  0  100;
        -1  0  0  120;
         0  0  0    1]

T_ad = [ 0  0 -1  400; 
         0 -1  0   50;
        -1  0  0  300;
         0  0  0    1]
     
T_bc = [ 0 -1/sqrt(2) -1/sqrt(2)  30; 
         0 1/sqrt(2)  -1/sqrt(2) -40;
         1  0  0  25;
         0  0  0    1]

% in order to visualize all reference frames I need to compute their
% position and orientation wrt the base frame T_a, therefore I need to:
T_ad = T_a*T_ad;
T_ab = T_ad*T_db;
T_ac = T_ab*T_bc;
T_ae = T_ad*T_de;
% now I can compute T_ce      
T_ce = TransInv(T_ac)*T_ae    

% this command creates a figure without any specifications
figure(1)      
% this command tells the figure to show a volume of 450 in length for each
% side:
plotvol(450)
% the command below sets the view angle
view(-120, 35)
% this command plots a HTM, in this case T_a, with axis length 50, and
% writes the letter 'a' next to it, and gives it a black color
trplot(T_a,'length',50, 'frame', 'a', 'color', 'k')
hold on
trplot(T_ad,'length',50, 'frame','d', 'color', 'b')
trplot(T_ab,'length',50, 'frame','b', 'color', 'g')
trplot(T_ac,'length',50, 'frame','c', 'color', 'r')
trplot(T_ae,'length',50, 'frame', 'e', 'color', 'm')
% in order to check if the calculation of T_ce is correct I express it wrt
% the base frame T_a and plot it. If it overlaps T_ae, then it means the
% solution is correct
trplot(T_ac*T_ce,'length', 100, 'frame', 'e', 'color', 'y')