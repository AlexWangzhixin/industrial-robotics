function [AdjointFrancesco] = Adjointme(T)
% this creates my own function to compute the adjoint of HTM
Tsb = T;
Adj = zeros(6,6);
Rsb = Tsb(1:3,1:3);
oRsb = VecToso3(Tsb(1:3,4)')*Rsb;
Adj(1:3,1:3) = Rsb;
Adj(4:6,1:3) = oRsb;
Adj(4:6, 4:6) = Rsb;
AdjointFrancesco = Adj;
return