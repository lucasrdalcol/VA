function T=homoTransform2d(p)

x = p(1);
y = p(2);
a = p(3);

T = [cos(a) -sin(a)  x
     sin(a)  cos(a)  y
          0       0  1];