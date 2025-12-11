% exercise on the calculation of the Adjoint operator
% given Tsb and Sb, compute Ss from Adjoint of Tsb

clear all
close all

Ts  = eye(4);
Rsb = roty(-pi/2);
psb = [0 2 0]';
Tsb = RpToTrans(Rsb,psb);
Sb  = [1 0 0 0 0 0]';
Ad_Tsb = Adjoint(Tsb)

Ss = Ad_Tsb*Sb

figure(1)
set(gcf,'position',[100,100, 700, 700])
plotvol(3)
hold on
view(60,23)
trplot(Ts,'frame' ,'s','color', 'k')
trplot(Tsb,'frame', 'b','color', 'b')
plot_arrow(Tsb(1:3,4), Tsb(1:3,4)+Ss(1:3),'r')
plot_arrow(Ts(1:3,4), Ss(4:end),'r')

% if Ss was given together with Tsb and we wanted to compute Sb
% then we need to compute the inverse of the Adjoint remmbering that:
% [Adjoint_Tsb]^-1 = [Adjoint_Tsb^-1]

Tbs = TransInv(Tsb);
Ad_Tbs = Adjoint(Tbs)
Sb = Ad_Tbs*Ss