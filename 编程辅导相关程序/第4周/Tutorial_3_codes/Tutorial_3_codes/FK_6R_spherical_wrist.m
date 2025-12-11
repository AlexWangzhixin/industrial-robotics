% 6R manipulator with spherical wrist (i.e. the last three joints attached
% to the end-effector are located in the same point)

clear all
close all
L1 = 0.3;
L2 = 0.2;
L3 = 0.2;

h    = 1.0;

theta1 = 0.0; % [rad] rotation of joint 1
omg1 = [0 0 1]';
q1   = [0 0 0]';
v1   = [-cross(omg1,q1)];
S1   = [omg1;v1];
S1_mat = VecTose3(S1);

theta2 = 0.0; % [rad] rotation of joint 2
omg2 = [0 -1 0]';
q2   = [0 0 L1]';
v2   = [-cross(omg1,q1)];
S2   = [omg2;v2];
S2_mat = VecTose3(S2);

theta3 = 0.0; % [mm] extension of joint 3
omg3 = [0 -1 0]';
q3   = [L2 0 L1]';
v3   = [-cross(omg3,q3)];
S3   = [omg3;v3];
S3_mat = VecTose3(S3);

theta4 = 0.0; % [mm] extension of joint 4
omg4 = [1 0 0]';
q4   = [L2+L3 0 L1]';
v4   = [-cross(omg4,q4)];
S4   = [omg4;v4];
S4_mat = VecTose3(S4);

theta5 = 0.0; % [mm] extension of joint 5
omg5 = [0 -1 0]';
q5   = [L2+L3 0 L1]';
v5   = [-cross(omg5,q5)];
S5   = [omg5;v5];
S5_mat = VecTose3(S5);

theta6 = 0.0; % [mm] extension of joint 6
omg6 = [1 0 0]';
q6   = [L2+L3 0 L1]';
v6   = [-cross(omg6,q6)];
S6   = [omg6;v6];
S6_mat = VecTose3(S6);

M1 = RpToTrans(eye(3), [0 0 0]');
M2 = RpToTrans(eye(3), [0 0 L1]');
M3 = RpToTrans(eye(3), [L2 0 L1]');
M4 = RpToTrans(eye(3), [L2+L3 0 L1]'); %
M5 = RpToTrans(eye(3), [L2+L3 0 L1]'); % these joint are part of the spherical joint
M6 = RpToTrans(eye(3), [L2+L3 0 L1]'); %
M  = RpToTrans(eye(3), [L2+L3 0 L1]'); % end-effector

figure(2)
x0=200;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=800;  % width of figure
height=800; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(L1+L2+L3)
view(75,29)
trplot(M1, 'length', L1/2)
hold on

figure(3)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=400; % height of figure
set(gcf,'position',[x0,y0,width,height])    % set figures specifications
plotvol(L1+L2+L3)
view(46,34)
trplot(M1, 'length', L1/2)
hold on

for i = 1:20

% to plot the full workspace loop over 2000 points and uncomment the next 6
% lines:
%     theta1 = rand*2*pi;
%     theta2 = rand*2*pi;
%     theta3 = rand*2*pi;
%     theta4 = rand*2*pi;
%     theta5 = rand*2*pi;
%     theta6 = rand*2*pi;
% if you want to plot a trajectory uncomment the following lines and
% comment the ones above:
    deltat = 0.1;
    t(i) = deltat*i;
    freq1  = 0.3;
    freq2  = 0.3;
    freq3  = 0.3;
    freq4  = 0.3;
    freq5  = 0.3;
    freq6  = 0.3;
    theta1 = theta1+freq1*deltat;
    theta2 = theta2+freq2*deltat;
    theta3 = theta3+freq3*deltat;
    theta4 = theta4+freq4*deltat;
    theta5 = theta5+freq5*deltat;
    theta6 = theta6+freq6*deltat;
    joint_angle1(i) = theta1;
    joint_angle2(i) = theta2;
    joint_angle3(i) = theta3;
    joint_angle4(i) = theta4;
    joint_angle5(i) = theta5;
    joint_angle6(i) = theta6;

    T0e = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*MatrixExp6(S3_mat*theta3)*MatrixExp6(S4_mat*theta4)*MatrixExp6(S5_mat*theta5)*MatrixExp6(S6_mat*theta6)*M;
    T06 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*MatrixExp6(S3_mat*theta3)*MatrixExp6(S4_mat*theta4)*MatrixExp6(S5_mat*theta5)*MatrixExp6(S6_mat*theta6)*M6;
    T05 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*MatrixExp6(S3_mat*theta3)*MatrixExp6(S4_mat*theta4)*MatrixExp6(S5_mat*theta5)*M5;
    T04 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*MatrixExp6(S3_mat*theta3)*MatrixExp6(S4_mat*theta4)*M4;
    T03 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*M3;
    T02 = MatrixExp6(S1_mat*theta1)*MatrixExp6(S2_mat*theta2)*M2;
    T01 = MatrixExp6(S1_mat*theta1)*M1;    

    Eex(i) = T0e(1,4); 
    Eey(i) = T0e(2,4); 
    Eez(i) = T0e(3,4);
    
    figure(2)
    plotvol(2*L1)
    view(46,34)
    trplot(T01, 'length', L1/3)
    trplot(T02, 'length', L1/3)
    trplot(T03, 'length', L1/3)
    trplot(T0e,'color','k', 'length', L1/3)
    plot3([T06(1,4) T0e(1,4)], [T06(2,4) T0e(2,4)], [T06(3,4) T0e(3,4)],'o-k')
    plot3([T03(1,4) T04(1,4)], [T03(2,4) T04(2,4)], [T03(3,4) T04(3,4)],'o-b')
    plot3([T01(1,4) T02(1,4)], [T01(2,4) T02(2,4)], [T01(3,4) T02(3,4)],'o-b')
    plot3([T02(1,4) T03(1,4)], [T02(2,4) T03(2,4)], [T02(3,4) T03(3,4)],'o-b')
    plot3(Eex, Eey, Eez, '.r')
    drawnow
    pause(0.001)
        
end

figure(3)
plot3(Eex, Eey, Eez, '.r')

figure(4)
plot(t, Eex)
hold on
plot(t, Eey)
plot(t, Eez)
xlabel('time [sec]')
ylabel('position [m]')
legend('x','y','z')



