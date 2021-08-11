function problem = randomPointCloudRegistration(N,varargin)
params = inputParser;
params.CaseSensitive = false;
params.addParameter('NoiseSigma',0.01,...
    @(x) isscalar(x));
params.parse(varargin{:});
NoiseSigma = params.Results.NoiseSigma;

X = randn(3,N);
R = rand_rotation;
t = randn(3,1);

Y = R * X + t + NoiseSigma * randn(3,N);


problem.type = 'PointCloudRegistration';
problem.N = N;
problem.model = X; % full point cloud
problem.scene = Y; % full point cloud
problem.X = X; % keypoints
problem.Y = Y; % keypoints
problem.R_gt = R;
problem.t_gt = t;
end