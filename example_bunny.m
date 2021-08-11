clc
clear
close all

addpath(genpath('./lib'))

bunny_fname     = './data/bun_zipper_res3.ply';

bunnypcd        = pcread(bunny_fname);

Axyz            = double(bunnypcd.Location)';
NoiseSigma      = 0.001;
Scale           = 0.5;
[Bxyz,R,t]      = RandomTransformPointCloud(Axyz,NoiseSigma,Scale);

problem.type    = 'PointCloudRegistration';
problem.N       = 100;
problem.model   = Axyz; % full point cloud
problem.scene   = Bxyz; % full point cloud
problem.X       = Axyz(:,1:problem.N); % keypoints
problem.Y       = Bxyz(:,1:problem.N); % keypoints
problem.R_gt    = R;
problem.t_gt    = t;

%% convert to standard primitive problem
standard        = StandardPrimitiveProblem(problem);

%% solve using DAMP
[R_est,t_est,info] = DAMP(standard,...
    'Damping',1.0,'TimeStep',0.3,'StopTh',1e-4,'SDamping',0.1);
R_err              = getAngularError(standard.R, R_est);
t_err              = getTranslationError(standard.t, t_est);

fprintf('DAMP: R_err: %g[deg], t_err: %g[m], time: %g[s].\n',...
    R_err,t_err,info.time);

%% plot and animate
saveVideo = false;
fileName = sprintf('/Users/hankyang/Documents/MATLAB/DAMP/Videos/sample_PCR.mp4');
PlotPointCloudRegistration(problem,info.XItstTraj,info.YItstTraj,info.StateTraj,...
    'SaveVideo',saveVideo,...
    'FileName',fileName);



function [B,R,t] = RandomTransformPointCloud(A,NoiseSigma,Scale)
N = size(A,2);
R = rand_rotation;
t = randn(3,1);
t = t/norm(t);
t = Scale*rand*t;

B = R*A + t + NoiseSigma * randn(3,N);
end