function [R_est, t_est, info] = DAMP(standard,varargin)
% DAMP: DynAMical Pose estimation
% Solve a standard problem of registering two sets of geometric primitives

params = inputParser;
params.CaseSensitive = false;

params.addParameter('Damping',1.0,...
    @(x) isscalar(x));

params.addParameter('SDamping',0.1,...
    @(x) isscalar(x));

params.addParameter('TimeStep',0.3,...
    @(x) isscalar(x));

params.addParameter('MaxIters',1e4,...
    @(x) isscalar(x));

params.addParameter('StopTh',1e-6,...
    @(x) isscalar(x));

params.addParameter('VariableStep',false,...
    @(x) islogical(x));

params.addParameter('EscapeEquilibrium',false,...
    @(x) islogical(x));

params.addParameter('EscapeTimes',5,...
    @(x) isscalar(x));

params.addParameter('PrintInfo',false,...
    @(x) islogical(x));

params.parse(varargin{:});

Damping = params.Results.Damping;
SDamping = params.Results.SDamping;
TimeStep = params.Results.TimeStep;
MaxIters = params.Results.MaxIters;
StopTh = params.Results.StopTh;
VariableStep = params.Results.VariableStep;
EscapeEquilibrium = params.Results.EscapeEquilibrium;
EscapeTimes = params.Results.EscapeTimes;
PrintInfo = params.Results.PrintInfo;

if VariableStep
    disp('DAMP using variable step size.')
end

N = standard.N;
P = standard.P;
Q = standard.Q;

fprintf('DAMP aligning %d primitives...\n',N);

if ~isfield(standard,'k')
    standard.k = ones(N,1);
end

X = zeros(3,N);
for i = 1:N
    X(:,i) = P{i}.x;
end

X_center = mean(X,2);
X_ref = X - X_center;

J = zeros(3,3); % moment of inertia
for i = 1:N
    temp = hatmap(X_ref(:,i));
    J = J - temp * temp;
end
RJ = chol(J); % Cholesky factor

% initialize state
xbar = X_center;
q = [0;0;0;1];
vbar = randn(3,1);
omega = randn(3,1);

s = [xbar;q;vbar;omega];

iter = 0;
StateTraj = {};
YItstTraj = {};
XItstTraj = {};

start = tic;

% tol = 1e-5;

if EscapeEquilibrium
    LocalMins = [];
    LocalSols = [];
    Trial = 1;
end

while iter < MaxIters
    [s_dot,X_itst,Y_itst] = DAMPDynamics(s,P,Q,X_ref,J,RJ,Damping,SDamping,...
        standard.type,standard.k);

    StateTraj{end+1} = s;
    YItstTraj{end+1} = Y_itst;
    XItstTraj{end+1} = X_itst;

    [Vp,Vk,V] = ComputeSystemEnergy(s,X_itst,Y_itst,J,standard.k);
    s_dot_norm = norm(s_dot);
    
    if PrintInfo
        if rem(iter,100) == 1
        fprintf('Iteration %d: dState = %g, Vp=%g, Vk=%g, V=%g.\n',...
            iter,s_dot_norm,Vp,Vk,V);
        end
    end

    if s_dot_norm < StopTh
        if EscapeEquilibrium && Trial < EscapeTimes+1
            fprintf('EscapeEquilibrium Trial %d.\n',Trial);
            s_dot = 2 * randn(length(s_dot),1);
            LocalMins = [LocalMins,Vp];
            LocalSols = [LocalSols,s];
            Trial = Trial + 1;
        else
            fprintf('Iteration %d: dState = %g, Vp=%g, Vk=%g, V=%g.\n',...
            iter,s_dot_norm,Vp,Vk,V);
            break;
        end
    end
    
%     if VariableStep
%         s_0 = UpdateState(s,s_dot,TimeStep);
%         s_1_2 = UpdateState(s,s_dot,TimeStep/2);
%         [s_dot_1_2,~,~] = DAMPDynamics(s_1_2,P,Q,X_ref,J,Damping,SDamping,standard.type);
%         s_1 = UpdateState(s_1_2,s_dot_1_2,TimeStep/2);
% 
%         tau = norm(s_1 - s_0);
% 
%         temp = max(sqrt(tol/(2*tau)),0.3);
%         scale = 0.9*min(temp,2);
%         TimeStep = TimeStep*scale;
%         
%         fprintf('Itr %d: tau=%g, scale=%g, next TimeStep=%g.\n',iter,tau,scale,TimeStep);
%         
%         s = s_0;
%     else
    s = UpdateState(s,s_dot,TimeStep);
%     end
    iter = iter + 1;
end

if EscapeEquilibrium
    [~,idx] = min(LocalMins);
    s = LocalSols(:,idx);
end

R_est = Quat2Rotm(s(4:7));
t_est = s(1:3) - R_est * X_center;

time = toc(start);

info.X_ref = X_ref;
info.StateTraj = StateTraj;
info.XItstTraj = XItstTraj;
info.YItstTraj = YItstTraj;
info.time = time;
info.f_cost = Vp;
end
