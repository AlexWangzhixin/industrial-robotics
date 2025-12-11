% given a vector of angular rotation computes the skew symmetric form of
% its own unit vector

clear all 
close all

omg = [1 2 3]'

[omg_hat, omg_mag] = AxisAng3(omg)

omg_hat_skew = VecToso3(omg_hat)