clc
clear
close all

% rng(0);

addpath(genpath('./lib'))

%% generate a random absolute pose estimation problem
N = 20;
problem = RandAbsolutePoseEstimation(N,'NoiseSigma',0.01);

%% convert it to standard primitive registration problem
standard = StandardPrimitiveProblem(problem);

%% solve using DAMP
[R_est,t_est,info] = DAMP(standard,...
    'Damping',1.0,'TimeStep',0.2,'StopTh',1e-3,'SDamping',0.1,...
    'VariableStep',false,...
    'EscapeEquilibrium',true);
R_err = getAngularError(standard.R, R_est);
t_err = getTranslationError(standard.t, t_est);

fprintf('DAMP: R_err: %g[deg], t_err: %g[m], time: %g[s].\n',...
    R_err,t_err,info.time);


%% visualize
% saveVideo = true;
% fileName = '/Users/hankyang/Documents/MATLAB/DAMP/Videos/sample_APE.mp4';
% PlotAbsolutePoseEstimation(problem,info.XItstTraj,info.YItstTraj,info.StateTraj,...
%     'SaveVideo',saveVideo,...
%     'FileName',fileName);


