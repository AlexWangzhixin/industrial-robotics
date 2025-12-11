% test matrix exponential 2
% we show three equivalent ways of computing the new rotation matrix after
% a rotation around the rotation vector omg = [0 0 2]

clear all
close all

% first we define the velocity vector omega, this represents an angular
% velocity around the axis z 
omg = [0 0 1.]

% we want to extract the amount of rotation theta, to do that we compute
% the absolute value, or norm, of vector omg
theta = sqrt(omg(1)^2+omg(2)^2+omg(3)^2)
theta =  norm(omg) % this is an equivalent command to the one above to compute the norm

% a nice way to include description in the code while you are executing it
% is to use the follwing command
disp('the amount of rotation around the axis is: ')
disp(rad2deg(theta)) % if you want to see the value of theta in degrees instead of radians

% then we want to compute the unit axis of rotation, omg_hat:
omg_hat = omg/theta
% and then we want to transfor that in skew-symmetric matrix form, because
% that is the form we need to compute the matrix exponential:
omg_mat = VecToso3(omg_hat)
% then we compute the matrix exponential
R01 = MatrixExp3(omg_mat*theta)

disp('-----------------------------------------')

% the same exercise above, we can solve equivalently as follows:
[omg_hat, theta] = AxisAng3(omg)

omg_mat = VecToso3(omg_hat)

R01 = MatrixExp3(omg_mat*theta)

disp('-----------------------------------')

% finally notice that we would get the same solution even if we did not
% separate omg into omg_hat and theta, as long as we compute the
% skew-symmetric form of omg and then do the exponential of that

if (omg(3)>1.5)
    disp('value is ok')
    omg_mat = VecToso3(omg)
    R01 = MatrixExp3(omg_mat)
else
    disp('-- WARNING --- non acceptable value')
    disp('-- WARNING --- non acceptable value')
    disp('-- WARNING --- non acceptable value')
    disp('-- WARNING --- non acceptable value')
    return
end