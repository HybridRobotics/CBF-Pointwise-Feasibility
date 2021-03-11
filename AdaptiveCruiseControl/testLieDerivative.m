%% lie derivative
syms p v z
syms m Fr v0

X = [p;
    v;
    z];

f = [v;
    -(1/m)*Fr;
    v0 - v];

g = [0;
    1/m;
    0];

h = z - 1.8*v;

Lfh = jacobian(h,X)*f
Lgh = jacobian(h,X)*g