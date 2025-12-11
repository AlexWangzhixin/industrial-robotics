function [Adj] = Adjoint1(T)
% this function takes as input a HTM and compute the adjoint
%          [         0  0  0 ]
%          [   R     0  0  0 ]
%   Adj =  [         0  0  0 ]
%          [                 ]
%          [  [d]*R      R   ]
%          [                 ]
%
[R,p] = TransToRp(T);
Adj = zeros(6,6); % this creates a matrix 6x6 of all zeros, which we then fill
Adj(1:3,1:3) = R; % this puts the rotation matrix R in the top left 3x3
Adj(4:6,1:3) = VecToso3(p)*R; % this puts the producte in the bottom left 3x3
Adj(4:6,4:6) = R; %this puts the rotation matrix in the bottom right 3x3
end