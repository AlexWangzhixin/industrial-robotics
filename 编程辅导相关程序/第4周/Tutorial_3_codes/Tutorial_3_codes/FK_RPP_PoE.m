% RPP cartesian manipulator
clear all
close all
L1 = 0.3;
L2 = 0.2;

h    = 1.0;

theta1 = 0.0; % [rad] rotation of joint 1
omg1 = [0 0 1]';
q1   = [0 0 0]';
v1   = [-cross(omg1,q1)];
S1   = [omg1;v1];
S1_mat = VecTose3(S1);

theta2 = L1; % [mm] extension of joint 2
omg2 = [0 0 1]';
q2   = [0 0 0]';
v2   = h*omg2;
S2   = [[0 0 0]';v2];
S2_mat = VecTose3(S2);

theta3 = L2; % [mm] extension of joint 3
omg3 = [1 0 0]';
q3   = [0 0 0]';
v3   = h*omg3;
S3   = [[0 0 0]';v3];
S3_mat = VecTose3(S3);

M  = RpToTrans(eye(3), [0 0 0]');
M1 = RpToTrans(eye(3), [0 0 0]');
M2 = RpToTrans(eye(3), [0 0 0]');
M3 = RpToTrans(eye(3), [0 0 0]');

% figure(1)
% trplot(M, 'length', L1/2)
% hold on
% trplot(M1, 'length', L1/2)
% trplot(M2, 'length', L1/2)
% trplot(M3, 'length', L1/2)
% plot3([M3(1,4) M(1,4)], [M3(2,4) M(2,4)], [M3(3,4) M(3,4)])
% plot3([M1(1,4) M2(1,4)], [M1(2,4) M2(2,4)], [M1(3,4) M2(3,4)])
% plot3([M2(1,4) M3(1,4)], [M2(2,4) M3(2,4)], [M2(3,4) M3(3,4)])

figure(2)
x0=100;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(L1)
view(46,34)
trplot(M1, 'length', L1/2)
hold on

figure(3)
x0=500;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(L1)
view(46,34)
trplot(M1, 'length', L1/2)
hold on

for i = 1:3000
   
    theta1 = rand*2*pi;
    theta2 = rand*L1;
    theta3 = rand*L2;
% if you want to plot a different trajectory uncomment the following lines
%     time   = i;
%     freq1  = 0.1;
%     freq2  = 0.1;
%     freq3  = 0.1;
%     theta1 = (2*pi)*sin(freq1*i);
%     theta2 = L1*abs(sin(freq2*i));
%     theta3 = L2*abs(sin(freq3*i));

    T0e = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*MatrixExp6(S3_mat*theta3)*M;
    T03 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*M3;
    T02 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*M2;
    T01 = MatrixExp6(S1_mat*theta1)*M1;    

    Eex(i) = T0e(1,4); 
    Eey(i) = T0e(2,4); 
    Eez(i) = T0e(3,4);
    
%     figure(2)
%     plotvol(2*L1)
%     view(46,34)
%     trplot(T0e, 'length', L1/2)
%     trplot(T01, 'length', L1/2)
%     trplot(T02, 'length', L1/2)
%     trplot(T03, 'length', L1/2)
%     plot3([T03(1,4) T0e(1,4)], [T03(2,4) T0e(2,4)], [T03(3,4) T0e(3,4)])
%     plot3([T01(1,4) T02(1,4)], [T01(2,4) T02(2,4)], [T01(3,4) T02(3,4)])
%     plot3([T02(1,4) T03(1,4)], [T02(2,4) T03(2,4)], [T02(3,4) T03(3,4)])
%     plot3(Eex, Eey, Eez, '.r')
%     drawnow
%     pause(0.05)
    
    
     
end

figure(3)
plot3(Eex, Eey, Eez, '.r')


