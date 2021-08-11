function problem = RandAbsolutePoseEstimation(num_corrs,varargin)
% generate a random absolute pose estimation problem

problem.type = 'AbsolutePoseEstimation';

params = inputParser;
params.CaseSensitive = false;

params.addParameter('NoiseSigma',0.01,...
    @(x) isscalar(x));

params.addParameter('SceneRadius',1.0,...
    @(x) isscalar(x));

params.parse(varargin{:});

NoiseSigma = params.Results.NoiseSigma;
SceneRadius = params.Results.SceneRadius;

% generate a 3D model
% X = SceneRadius*randn(3,num_corrs);
X_c = [-2 + 4*rand(1,num_corrs);
       -2 + 4*rand(1,num_corrs);
       4 + 4*rand(1,num_corrs)];

% generate rotation and translation
R = rand_rotation;
t = SceneRadius*randn(3,1); 

% generate projections
x_h = X_c ./ X_c(3,:);
x = x_h(1:2,:);

% add noise
x = x + NoiseSigma * randn(2,num_corrs);

X = R * X_c + t;

R_gt = R';
t_gt = - R'*t;

problem.X = X;
problem.x = x;
problem.R_gt = R_gt;
problem.t_gt = t_gt;
problem.N = num_corrs;
end