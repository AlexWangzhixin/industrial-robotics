clear all
close all
format short
T_s = eye(4);
q = [-2; 0; 0];
omghat = [0; 1; 0];
vhat = -cross(omghat,q)
theta = deg2rad(10);
Ss = [omghat;vhat]
Ss_mat = VecTose3(Ss)
%T_sb = MatrixExp6(Ss_mat*theta)

figure(1)
plotvol(5)
hold on
view(144,30)
trplot(T_s, 'frame', 's', 'color', 'k', 'arrow')
hold on
plot_arrow(q, q+Ss(1:3), 'r')
%trplot(T_sb,'frame', 'b', 'color', 'b', 'arrow')
T_temp = T_s;
n = 20
for i=1:n
    theta = theta+theta*deg2rad(10);
    T_temp = MatrixExp6(Ss_mat*theta);
    trplot(T_temp,'frame', 'b', 'color', 'b', 'linewidth', 0.5)
    plot_arrow(q, T_temp(1:3,4))
    pause(0.2)
end