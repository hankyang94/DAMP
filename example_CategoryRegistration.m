clc
clear
close all

addpath(genpath('./lib'))

problem.type = 'CategoryRegistration';
problem.class = 'chair';
problem.model = ComputeChairClass('Conf',0.5);
R = rand_rotation;
t = randn(3,1);

num_models = size(problem.model.cadmodels,1)/3;
idx = 2;
scene = problem.model.cadmodels(blkIndices(idx,3),:);
scenecloud = problem.model.clouds{idx};
scenecloud = scenecloud - mean(scene,2);
scene = scene - mean(scene,2);
problem.scene = R*scene + t;
problem.scenecloud = R*scenecloud + t;
problem.N = size(scene,2);
problem.R_gt = R';
problem.t_gt = -R'*t;

%% convert to standard primitive alignmen
standard = StandardPrimitiveProblem(problem);

%% solve using DAMP
[R_est,t_est,info] = DAMP(standard,...
    'Damping',1.0,'TimeStep',0.3,'StopTh',1e-3,'SDamping',0.1);
R_err = getAngularError(standard.R, R_est);
t_err = getTranslationError(standard.t, t_est);

fprintf('DAMP: R_err: %g[deg], t_err: %g[m], time: %g[s].\n',...
    R_err,t_err,info.time);

%% plot and animate
saveVideo = false;
fileName = sprintf('/Users/hankyang/Documents/MATLAB/DAMP/Videos/sample_CR_%s.mp4',problem.class);
PlotCategoryRegistration(problem,info.XItstTraj,info.YItstTraj,info.StateTraj,...
    'SaveVideo',saveVideo,...
    'FileName',fileName);
