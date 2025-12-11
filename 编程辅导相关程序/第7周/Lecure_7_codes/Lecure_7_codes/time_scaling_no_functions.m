%time scaling motion example: this code defines the duration of the
%trajectory and the number of timesteps in which the trajectory is divided,
%then uses a cubic timescaling like the one seen in class to compute the
%parameter "s" and plots s and its derivative to demonstrate what this
%parameter looks like with respect to time
clear all
close all

tend = 5;
N = 140;
deltat = tend/N;
s = zeros(N,1);

for i=1:N
   s(i) = CubicTimeScaling(tend,deltat*i);
   time(i) = deltat*i;
end
dot_s=s*0;
dot_s(2:end) = (s(2:end)-s(1:end-1))/deltat;
figure(1)
plot(time,s,'b')
hold on
plot(time,dot_s,'r')
legend('$s$', '$\dot s$','interpreter', 'latex')
xlabel('time [sec]','interpreter', 'latex')
ylabel('position and velocity','interpreter', 'latex')
title('trajectory cubic time scaling','interpreter', 'latex')