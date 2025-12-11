% an example of the usage of angle and axis screw elements
% if we have a frame {s} and an angular velocity omega, we can immediately
% compute how {s} will look like after having rotated around omega 

T_s = eye(4);  % generate the fixed frame {s}
omega = [1;3;1]; % generate the angular velocity axis of rotation

% first we compute the skew symmetric form of the unit axis omega
omega_mat = VecToso3(omega)
% then we compute the matrix exponential of omega_mat and theta
T_b = MatrixExp3(omega_mat)

figure(1)
x0=800;     % location of figure in the screen
y0=200;     % location of figure in the screen
width=400;  % width of figure
height=300; % height of figure
set(gcf,'position',[x0,y0,width,height])
plotvol(max(omega))
hold on
trplot(T_s,'frame','s','arrow','color','k')
plot_arrow([0;0;0], omega, 'r')
trplot(T_b,'frame','b', 'arrow','color','b')