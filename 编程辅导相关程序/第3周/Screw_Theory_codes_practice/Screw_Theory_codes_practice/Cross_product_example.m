% this is an example of using the cross product function
clear all
close all

a = [1 1 0]
b = [0 1 1]

c = cross(a, b)

figure(1)
plotvol(2)
view(149, 23)
trplot(eye(4), 'arrow', 'length', 0.5, 'color', 'k')
hold on
plot_arrow([0 0 0], a, 'r')
plot_arrow([0 0 0], b , 'b')
plot_arrow([0 0 0], c , 'g')