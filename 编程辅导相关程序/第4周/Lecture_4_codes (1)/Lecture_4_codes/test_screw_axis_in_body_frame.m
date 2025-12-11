% in this example we define the screw axes in the fixed frame and transform
% them in the body frame by using the adjoint operator

clear all
close all

L1 = 1;
L2 = 1;
L3 = 1;

Ts = eye(4);                             % fixed frame
M = RpToTrans(eye(3), [L1+L3; 0; -L2])   % end-effector body frame

% if I had S1 and wanted B1 I would do:
S1 = [0 0 1 0 0 0]'
B1 = Adjoint(TransInv(M))*S1
% then we can try and verify what B3 looks like if I have S3
S3 = [1 0 0 -cross([1 0 0]', [0 0 -L2])]'
B3 = Adjoint(TransInv(M))*S3
% then we do the same with S2 and B2
S2 = [0 -1 0 -cross([0 -1 0]', [L1 0 0])]'
B2 = Adjoint(TransInv(M))*S2

% if we wanted to compute S1, by knowing B1, using the adjoint
B1 = [0 0 1 0 L1+L3 0]'
S1 = Adjoint(M)*B1      % what S1 looks like when I am looking at it from Ts