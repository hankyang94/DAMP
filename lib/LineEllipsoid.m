function [p_l,p_e] = LineEllipsoid(x,v,mu,A)
y = x-mu;
a = v'*A*v;
b = 2*y'*(A*v);
c = y'*A*y - 1;
% A_inv = inv(A);

dis = b^2 - 4*a*c;
if dis >= 0 % the line intersects the ellipsoid once or twice
    alpha = -0.5 * b/a;
    p = x + alpha*v;
    p_l = p;
    p_e = p;
else
    R = eye(3) - v*v';
    u = y - (v'*y)*v;
    lambda = 1e-6;
    itr = 0;
    MaxIters = 1e3;
    while itr < MaxIters
        [f,g] = G(lambda,A,R,u);
        if abs(f) < 1e-6
%             fprintf('LineEllipsoid done in %d iterations.\n',itr);
            break;
        end
        lambda = lambda - f/g;
        itr = itr + 1;
    end
    Q = lambda*A+R;
    p_e = Q\u + mu;
    alpha = v'*(p_e - mu - y);
    p_l = y + alpha*v + mu;
%     n = cross(x-mu,v);
%     n = n/norm(n);
%     g = cross(n,v);
%     lam = sqrt(g'*A_inv*g);
%     z = 1/lam * A_inv * g;
%     w1 = z + mu;
%     w2 = -z + mu;
%     w = [w1,w2];
%     alpha = v'*(w-x);
%     p = x + v * alpha;
%     
%     distance = sum((p-w).^2,1);
%     
%     [~,idx] = min(distance);
%     
%     p_l = p(:,idx);
%     p_e = w(:,idx);
end
end


function [f,g] = G(lambda,A,R,u)
Q = lambda * A + R;
Qinv = inv(Q);
w = Qinv*u;
temp = A*w;
f = w'*temp - 1.0;
g = -2*temp'*Qinv*temp;
end