function [y1,y2] = myfunction(x,c1,c2)
%This function takes a vector of coordinates x and two scalars c1 and c2 as inputs
% It outputs the y coordinates associated to 2 different trajectories of 
% a cooking robotic arm.

y1 = sin(c1*x).^2*c2; %calculates the y coordinates for the 1st trajectory.
y2 = cos(c1*x).^2/sqrt(c2); %calculates the y coordinates for the 2nd trajectory.

figure(); %creates a figure window
plot(x,y1); % plot x versus y1
hold on; plot(x,y2); %'hold on is used to tell Matlab to keep ploting on the 
%same graph.
grid on; %Used to display a grid on the graph.

%Define the titles of the x and y axes and the title of the graph
xlabel('x coordinates (unit)'); ylabel('y coordinates (unit)');
title('Trajectories of the end effector of a robotic cooking arm');
legend('trajectory 1', 'trajectory 2'); %Creates a legend for the plot

end

