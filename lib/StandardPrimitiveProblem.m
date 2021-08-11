function standard = StandardPrimitiveProblem(problem)
% convert different types of supported problem to standard problem of
% registering two sets of geometric primitives
switch problem.type
    case 'PointCloudRegistration'
        N = problem.N;
        X = problem.X;
        Y = problem.Y;
        
        P = {};
        Q = {};
        for i = 1:N
            p.type = 'point';
            p.x = X(:,i);
            
            q.type = 'point';
            q.x = Y(:,i);
            P{end+1} = p;
            Q{end+1} = q;
        end
        standard.type = 'PointCloudRegistration';
        standard.P = P;
        standard.Q = Q;
        standard.N = length(P);
        standard.R = problem.R_gt;
        standard.t = problem.t_gt;
    
    case 'PrimitiveRegistration'
        N = problem.N;
        P = {};
        Q = {};
        for i = 1:N
            p.type = 'point';
            p.x = problem.scene(:,i);
            
            q = problem.primitives{i};
            P{end+1} = p;
            Q{end+1} = q;
        end
        standard.type = 'PrimitiveRegistration';
        standard.P = P;
        standard.Q = Q;
        standard.N = length(P);
        standard.R = problem.R_gt;
        standard.t = problem.t_gt;
            
    case 'AbsolutePoseEstimation'
        N = problem.N; % number of correspondences
        X = problem.X; % 3D points
        x = problem.x; % 2D projections
       
        P = {};
        Q = {};
        for i = 1:N
            p.type = 'point';
            p.x = X(:,i);
            
            q.type = 'line';
            q.x = zeros(3,1);
            v = [x(:,i);1.0];
            v = v/norm(v);
            q.v = v;
            
            P{end+1} = p;
            Q{end+1} = q;
%             k = [k;1.0];
        end
        
%         if isfield(problem,'source') && problem.source == "SPEED"
%             disp('Add point-plane correspondences for SPEED.')
%             P = {};
%             Q = {};
%             for i = 1:N
%                 p.type = 'point';
%                 p.x = X(:,i);
%                 
%                 vi = [x(:,i);1.0]; vi = vi/norm(vi);
%                 for j = 1:N
%                     if j ~= i
%                         q.type = 'plane';
%                         q.x = zeros(3,1);
%                         
%                         vj = [x(:,j);1.0]; vj = vj/norm(vj);
%                         n = cross(vi,vj); n = n/norm(n);
%                         q.n = n;
% 
%                         P{end+1} = p;
%                         Q{end+1} = q;
% %                         k = [k;2.0];
%                     end
%                 end
%             end
%         end
        
        standard.type = 'AbsolutePoseEstimation';
        standard.P = P;
        standard.Q = Q;
        standard.N = length(P);
        standard.R = problem.R_gt;
        standard.t = problem.t_gt;
%         standard.k = k;
    case 'CategoryRegistration'
        N = problem.N;
        scene = problem.scene;
        model = problem.model;
        
        P = {};
        Q = {};
        k = zeros(N,1);
        
        for i = 1:N
            p.type = 'point';
            p.x = scene(:,i);
            
            q.type = 'ellipsoid';
            q.mu = model.mu(:,i);
            q.omega = model.omega(:,:,i);  
            P{end+1} = p;
            Q{end+1} = q;
            % make the spring constant inversely proportional to the volume of the ellipsoid
            % the larger the ellipsoid, the less confident in that
            % correspondence
            k(i) = det(q.omega)^(0.5); 
        end
        standard.type = 'CategoryRegistration';
        standard.P = P;
        standard.Q = Q;
        standard.N = N;
        standard.R = problem.R_gt;
        standard.t = problem.t_gt;
        k = k/sum(k) * N; % the spring constants sum to N (1 per correspondence)
        standard.k = k;
        
    case 'CategoryAbsolutePoseEstimation'
        N = problem.N;
        x = problem.x; % 2D projections
        model = problem.model; % ellipsoid model
        P = {};
        Q = {};
        for i = 1:N
            p.type = 'ellipsoid';
            p.x = model.mu(:,i); % rename from mu to x
            p.A = model.omega(:,:,i); % rename from omega to A
            
            q.type = 'line';
            q.x = zeros(3,1);
            v = [x(:,i);1.0];
            v = v/norm(v);
            q.v = v;
            P{end+1} = p;
            Q{end+1} = q;
        end
        standard.type = 'CategoryAbsolutePoseEstimation';
        standard.P = P;
        standard.Q = Q;
        standard.N = N;
        standard.R = problem.R_gt;
        standard.t = problem.t_gt;
        
    otherwise
        error('Unsupported problem type.')
        
end

end