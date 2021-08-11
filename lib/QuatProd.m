function c = QuatProd(a,b)
A = [a(4), -a(3), a(2), a(1);
     a(3), a(4), -a(1), a(2);
     -a(2), a(1), a(4), a(3);
     -a(1), -a(2), -a(3), a(4)];
c = A*b;
end