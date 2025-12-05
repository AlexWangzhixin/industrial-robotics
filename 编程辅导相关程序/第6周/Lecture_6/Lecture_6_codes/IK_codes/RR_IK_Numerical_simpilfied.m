clear all
close all
format short

%RRR jacobian and inverse kinematics
theta1=deg2rad(0);
theta2=deg2rad(0);
L1 = 1;
L2 = 1;

Ts =  eye(4);
M3 = RpToTrans(eye(3), [L1+L2 0 0]');
M2 = RpToTrans(eye(3), [L1 0 0]');
M1 = RpToTrans(eye(3), [0 0 0]');

omghat1 = [0 0 1]';
omghat2 = [0 0 1]';
q1 = [0 0 0]';
q2 = [L1 0 0]';
vhat1 = -cross(omghat1, q1);
vhat2 = -cross(omghat2, q2);
S1 = [omghat1;vhat1];
S2 = [omghat2;vhat2];
S1mat = VecTose3(S1);
S2mat = VecTose3(S2);

% here starts the IK numerical part
% assign desired position of the end-effector
xpos = 1.2;
ypos = 0.7;
xd = [xpos  ypos]';
theta1_guess = deg2rad(0); 
theta2_guess = deg2rad(0);
theta_guess = [theta1_guess; theta2_guess]

volume_length = 2; % this defines how big is the area we want to plot
figure(2)
grid on
plotvol([-volume_length volume_length+0.5 -1.5 1.5])
view([0,90])
trplot(Ts)
hold on
plot_sphere([xpos; ypos; 0], 0.05,  'r');

e = [100 100]';
tolerance = 0.001;
h = 0;
while norm(e)>tolerance
    h = h+1;  
    Ts3 = MatrixExp6(S1mat*theta_guess(1))*MatrixExp6(S2mat*theta_guess(2))*M3;
    Ts2 = MatrixExp6(S1mat*theta_guess(1))*MatrixExp6(S2mat*theta_guess(2))*M2;
    Ts1 = MatrixExp6(S1mat*theta_guess(1))*M1;
    xe = Ts3(1:2,4);
    e = (xd - xe);
    norm(e);
    fprintf(strcat('attempt number: ', num2str(h), '\n'))
    fprintf(strcat('error: ', num2str(norm(e)), '[m] \n'))
    fprintf('---------------------------------------------------------\n')
    
    Janalytics = [-L1*sin(theta_guess(1))-L2*sin(theta_guess(1)+theta_guess(2)) -L2*sin(theta_guess(1)+theta_guess(2));...
     L1*cos(theta_guess(1))+L2*cos(theta_guess(1)+theta_guess(2)) L2*cos(theta_guess(1)+theta_guess(2))];
    
    theta_guess = theta_guess + pinv(Janalytics)*e;
    
    % the command below plots each attempt to solve the IK. Normally you
    % would not show this, you only retain the final result
    figure(2)
    link1 = plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'o-.k','linewidth',0.5);
    hold on
    link2 = plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'o-.k','linewidth',0.5);
    plot(Ts3(1,4), Ts3(2,4), 'b.')
    hold on
    
    % the command below dictates that the iteration process is forced to finish after 100 attempts
    if (h > 100) | norm(e)<1.5*(norm(xd)-(L1+L2)) 
        break
    end    
end

fprintf(strcat('The required joint angles [deg] to solve the IK are:  \n'))
rad2deg(wrapTo2Pi(theta_guess)) % this just re-expresses the angle in values between 0 and 2*pi

figure(2)
link1 = plot3([Ts1(1,4) Ts2(1,4)],[Ts1(2,4) Ts2(2,4)],[Ts1(3,4) Ts2(3,4)], 'o-k','linewidth',2);
hold on
link2 = plot3([Ts2(1,4) Ts3(1,4)],[Ts2(2,4) Ts3(2,4)],[Ts2(3,4) Ts3(3,4)], 'o-k','linewidth',2);
plot(Ts3(1,4), Ts3(2,4), 'r.')
hold on
