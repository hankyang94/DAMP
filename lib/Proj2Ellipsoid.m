function xbar = Proj2Ellipsoid(y,mu,omega)
% given a 3D point y, project it onto the ellipsoid:
% (x-mu)'*omega*(x-mu) = 1.0
% Using the method in:
% ALGORITHMS OF PROJECTION OF A POINT ONTO AN ELLIPSOID
% If y already in the ellipsoid, then return y itself

u = y - mu;

temp = u'*omega*u - 1.0;

if temp <= 0 % inside ellipsoid
    xbar = y;
else
    lambda = 0;
    [V,D] = eig(omega);
    v = V'*u;
    itr = 0;
    MaxIters = 1e3;
    while itr < MaxIters
        [f,g] = G(lambda,D,v);
        if abs(f) < 1e-6
%             fprintf('Proj2Ellipsoid done in %d iterations.\n',itr);
            break;
        end
        lambda = lambda - f/g;
        itr = itr + 1;
    end
    a = lambda * diag(D) + ones(3,1);
    a_inv = 1.0 ./ a;
    w = diag(a_inv) * v;
    z = V*w;
    xbar = z + mu;
end

end


function [f,g] = G(lambda,D,v)
    d = diag(D);
    a = lambda*d + ones(3,1);
    a_inv = 1.0 ./ a;
    A = diag(a);
    A_inv = diag(a_inv);
    
    w = A_inv*v;
    temp = D*w;
    f = w'*temp - 1.0;
%     dGdA = -2 * (w*w'*D*A_inv);
    
    g = -2* temp'*A_inv*temp;
end