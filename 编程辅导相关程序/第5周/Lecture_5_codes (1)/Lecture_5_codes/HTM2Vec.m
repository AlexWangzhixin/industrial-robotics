% this function takes a HTM as input and returns a 6x1 vector defined as:
%       |  x  |
%       |  y  |
%       |  z  |
%  x_e= |alpha|      
%       |beta |
%       |gamma|
% where x,y,z are the cartesian coordinate (in units of distance, i.e. [m])
% and alpha, beta, gamma are the XYZ Euler angles (in [deg]) of the original HTM

function x=HTM2Vec(T)
[R,p] = TransToRp(T);
x(1:3) = p;
eul = rotm2eul(R, 'ZYX');
x(6) = rad2deg(eul(1));
x(5) = rad2deg(eul(2));
x(4) = rad2deg(eul(3));
x=x';
end