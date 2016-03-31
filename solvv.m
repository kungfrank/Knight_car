a = sym('a', 'real');
b = sym('b', 'real');
c = sym('c', 'real');
d = sym('d', 'real');

i = sym('i', 'real');
m = sym('m', 'real');
v = sym('v', 'real');
t = sym('t', 'positive');

eq1 = d == i;
eq2 = (a*t^3+b*t^2+c*t+d == 0);
eq3 = (3*a*m^2+2*b*m+c == 0);
eq4 = (a*m^3+b*m^2+c*m+d == v);

[sola, solb, solc, sold] = solve(eq1,eq2,eq3,eq4,[a,b,c,d]);

simplify(sola)
simplify(solb)
simplify(solc)