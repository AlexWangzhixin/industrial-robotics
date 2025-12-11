% example code on how to compute a rigid body motion by using the matrix exponential of a twist
% assuming we have an initial frame {s} (with its own orientation and position)
% and we know the position (so the vector q) and orientation (i.e. the vector omega_hat)
% of a screw axis that represents a revolute joint

T_s = eye(4); % initial pose of {s}
q = [0.5; 1;0.2] % position of a point on the screw axis
omega = [0.2;0.4;0.5] % angular velocity vector
% compute the non-dimensional value of the angular velocity, which is the 
% unit axis of rotation omega_hat and the amount of rotation theta
[omega_hat, theta] = AxisAng3(omega)
% then compute the unit linear velocity, remember the formula:
% v_hat = - omega_hat x q + h*omega_hat, with h=0 because a revolute joint has h=0  
v_hat = -cross(omega_hat, q)

S_s = [omega_hat; v_hat]
% an alternative way to compute S_s is by using the following expression:
h=0;
S_s = ScrewToAxis(q,omega_hat,h)
% compute the skew symmetric form of S_s
S_s_mat = VecTose3(S_s)

T_sb = MatrixExp6(S_s_mat*theta)
figure(1)
plotvol(2)
view(110,23)
hold on
trplot(T_s,'frame','s','arrow','color','k')
trplot(T_sb,'frame','b', 'arrow','color','b')
plot_arrow(q, q+S_s(1:3),'g')
T_temp = T_s;
N = 20; %number of images I want to create during the transformation
for i=1:N
   T_temp = T_temp*MatrixExp6(S_s_mat*theta/N); % computes the matrix exponential of S_sb for rotation theta
   trplot(T_temp,'length', 0.5, 'color', 'r', 'Linewidth', 0.5)
   pause(0.1)
end