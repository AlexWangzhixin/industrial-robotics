function [Jacob] = jacobian(Slist,thetalist)

n = max(size(thetalist))
Jacob = zeros(6,n)
Tsn = eye(4);
for i=2:n
    Tsn =  Tsn*MatrixExp6(VecTose3(Slist(:,i-1))*thetalist(i-1))
    Jacob(:,i) = Adjoint(Tsn)*Slist(:,i) 
end
return 
end