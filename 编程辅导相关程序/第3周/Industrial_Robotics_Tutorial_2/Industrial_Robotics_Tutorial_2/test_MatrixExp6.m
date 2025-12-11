% given q and omg find Tsb

q = [2 0 0]
omg = [0 0 3]
h = 3
[omg_hat, theta] = AxisAng3(omg)

S = [omg_hat'; -cross(omg_hat,q')'+omg_hat'*h]
S_mat = VecTose3(S)

Tsb = MatrixExp6(S_mat*theta)