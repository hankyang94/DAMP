clc
clear
close all

warning('off','all')

addpath(genpath('./lib'))

S = load('./data/cadmodels.mat','cadmodels','edge');

problem.type = 'CategoryAbsolutePoseEstimation';
problem.model = ComputeCarClass('Conf',0.7);
R = rand_rotation;
t = randn(3,1);

idx = 4;
scene = S.cadmodels(blkIndices(idx,3),:);
scene = scene - mean(scene,2);
N = size(scene,2);
NoiseSigma = 0;
temp = R * scene;
min_z = min(temp(3,:)); 
% make sure the model will be in front of the camera
t(end) = max(t(end), - min_z + 2.0);
% generate projections
X_c = R * scene + t;
x_h = X_c ./ X_c(3,:);
x = x_h(1:2,:);
% add noise
x = x + NoiseSigma * randn(2,N);

problem.N = N;
problem.x = x;
problem.R_gt = R;
problem.t_gt = t;

%% convert to stand primitive problem
standard = StandardPrimitiveProblem(problem);

%% solve using DAMP
[R_est,t_est,info] = DAMP(standard,...
    'Damping',1.0,'TimeStep',0.3,'StopTh',1e-3,'SDamping',0.1);
R_err = getAngularError(standard.R, R_est);
t_err = getTranslationError(standard.t, t_est);

fprintf('DAMP: R_err: %g[deg], t_err: %g[m], time: %g[s].\n',...
    R_err,t_err,info.time);

%% visualize
saveVideo = true;
fileName = '/Users/hankyang/Documents/MATLAB/DAMP/Videos/sample_CAPE.mp4';
PlotCategoryAbsolutePoseEstimation(problem,info.XItstTraj,info.YItstTraj,info.StateTraj,...
    'SaveVideo',saveVideo,...
    'FileName',fileName);

