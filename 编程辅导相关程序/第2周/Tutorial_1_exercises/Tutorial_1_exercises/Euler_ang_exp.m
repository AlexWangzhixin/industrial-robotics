% - Industrial Robotics Tutorial 1 -
% This exercise gives you a rotation matrix R_sb and shows how to compute
% the Euler angles in the format Z-Y-Z (that is in the current frame)    
% or in the format X-Y-Z (this is the roll-pitch-yaw format expressed wrt 
% fixed frame

clear all % this command closes all previous dataset and figures
close all 

R_s  = eye(3);
R_sb = [0.342 -0.9397  0;
        0.8138 0.2962 -0.5;
        0.4698 0.1710  0.866];
  
% computes euler angles    
EulZYZ = rad2deg(rotm2eul(R_sb, 'ZYZ')) % rotation in the current frame
EulXYZ = rad2deg(rotm2eul(R_sb, 'XYZ')) % rotation in the fixed frame

% double check if euler angles are correct by re-computing the rotation
% matrix from the euler angles.
A=rotz(67,'deg')*roty(-28,'deg')*rotx(11,'deg')
B=rotx(30,'deg')*roty(0,'deg')*rotz(70,'deg')