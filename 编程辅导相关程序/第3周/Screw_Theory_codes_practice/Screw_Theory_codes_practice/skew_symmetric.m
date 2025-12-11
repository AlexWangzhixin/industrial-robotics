function c = skew_symmetric(a)
% this function takes a UNIT vector a and outputs a skew symmetric matrix
% form of the same vector
c = [0    -a(3)  a(2);
     a(3)   0   -a(1);
    -a(2)  a(1)   0   ];
end