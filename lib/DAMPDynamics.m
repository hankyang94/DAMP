function [s_dot,X_itst,Y_itst] = DAMPDynamics(s,P,Q,X_ref,J,RJ,Damping,SDamping,type,springs)
% Given current state s, output derivative of s

N = size(X_ref,2);
% seperate the states
xbar = s(1:3); % location of CM
q = s(4:7); % attitude of P in unit quaternion
vbar = s(8:10); % linear velocity
omega = s(11:13); % angular velocity
R = Quat2Rotm(q);

f_spring = zeros(3,N);

X = R*X_ref + xbar;

X_itst = zeros(3,N); % hinge points in X
Y_itst = zeros(3,N); % hinge points in Y

for i = 1:N
    x = P{i};
    y = Q{i};
    if x.type == "point" && y.type == "point"
        xi = X(:,i);
        y_itst = y.x;
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
    
    elseif x.type == "point" && y.type == "line"
        xi = X(:,i);
        yi = y.x;
        vi = y.v;
        alpha = xi'*vi - yi'*vi;
        y_itst = yi + alpha * vi;
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
        
        % if the problem is camera pose estimation,
        % then enforce the 3D points to be in front of camera image plane
        if type == "AbsolutePoseEstimation"
            if xi(end) < 1.0 % the point must lie in front of the camera
                f_spring(:,i) = f_spring(:,i) + 5 * (xi(end) - 1.0) * [0;0;-1];
            end
        end
    elseif x.type == "point" && y.type == "plane"
        xi = X(:,i);
        yi = y.x;
        ni = y.n;
        alpha = ni'*(yi-xi);
        y_itst = xi + alpha * ni;
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
        % if the problem is camera pose estimation,
        % then enforce the 3D points to be in front of camera image plane
        if type == "AbsolutePoseEstimation"
            if xi(end) < 1.0 % the point must lie in front of the camera
                f_spring(:,i) = f_spring(:,i) + 5 * (xi(end) - 1.0) * [0;0;-1];
            end
        end
        
    elseif x.type == "point" && y.type == "sphere"
        xi = X(:,i);
        yi = y.x;
        r = y.r;
        yxi_diff = xi - yi;
        yxi_diff_norm = norm(yxi_diff);
        if yxi_diff_norm < 1e-6
            temp = randn(3,1);
            temp = temp/norm(temp);
            y_itst = yi + r * temp;
        else
            y_itst = yi + r * yxi_diff/yxi_diff_norm;
        end
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
        
    elseif x.type == "point" && y.type == "cylinder"
        xi = X(:,i);
        yi = y.x;
        vi = y.v;
        r = y.r;
        
        alpha = vi'*(xi-yi);
        yhat = yi + alpha*vi;
        
        x_diff_yhat = xi - yhat;
        x_diff_yhat_norm = norm(x_diff_yhat);
        
        if x_diff_yhat_norm < 1e-6
            temp = randn(3,1); temp = temp/norm(temp);
            u = cross(temp,vi); u = u/norm(u);
            y_itst = yhat + r * u;
        else
            y_itst = yhat + r * x_diff_yhat / x_diff_yhat_norm;
        end
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
        
    elseif x.type == "point" && y.type == "cone"
        xi = X(:,i);
        yi = y.x;
        vi = y.v;
        theta = y.theta;
        
        x_diff_y = xi - yi;
        
        if vi'*x_diff_y <= - norm(x_diff_y)*sin(theta)
            y_itst = yi;
        elseif norm(x_diff_y/norm(x_diff_y) - vi) < 1e-6
            temp = randn(3,1); temp = temp/norm(temp);
            perp = cross(vi,temp); perp = perp/norm(perp);
            angle = theta;
            Rtheta = axang2rotm([perp',angle]);
            u = Rtheta * vi;
            y_itst = yi + norm(x_diff_y)*cos(theta)*u;
        else
            axis = cross(vi,x_diff_y/norm(x_diff_y));
            axis = axis /norm(axis);
            angle = theta;
            Rtheta = axang2rotm([axis',angle]);
            w = Rtheta * vi;
            alpha = w'*x_diff_y;
            y_itst = yi + alpha*w;
        end
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
        
    elseif x.type == "point" && y.type == "ellipsoid"
        xi = X(:,i);
        mu = y.mu;
        OMEGA = y.omega;
        y_itst = Proj2Ellipsoid(xi,mu,OMEGA);
        f_spring(:,i) = 2 * springs(i) * (y_itst - xi);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = xi;
    elseif x.type == "ellipsoid" && y.type == "line"
        mui = X(:,i); % center of each ellipsoid
        Ai = R*x.A*R';
        [y_itst,x_itst] = LineEllipsoid(y.x,y.v,mui,Ai);
        
        f_spring(:,i) = 2 * springs(i) * (y_itst - x_itst);
        Y_itst(:,i) = y_itst;
        X_itst(:,i) = x_itst;
        if type == "CategoryAbsolutePoseEstimation"
            if mui(end) < 1.0 % the point must lie in front of the camera
                f_spring(:,i) = f_spring(:,i) + 5 * (mui(end) - 1.0) * [0;0;-1];
            end
        end
        
    else
        error('To be continued.')
    end
end

vel = R*(hatmap(omega)*X_ref) + vbar;

vel_norm = sum(vel.^2,1);

f_damping = -1 * Damping * vel - SDamping * (vel_norm .* vel);

f_total = f_spring + f_damping;
    
acc_bar = sum(f_total,2) / N;

q_dot = 0.5 * QuatProd(q,[omega;0]);

f_total_X = R' * f_total;

% This needs to be changed
% tau_X = cross(X_ref,f_total_X);
tau_X = cross(R'*(X_itst-xbar), f_total_X);

tau = sum(tau_X,2);

domega = RJ\(RJ' \ (tau - hatmap(omega)*(J*omega)));

s_dot = [vbar;q_dot;acc_bar;domega];
end