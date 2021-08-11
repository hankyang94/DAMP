clc
clear
close all


addpath(genpath('./lib'))

problem = RobotPrimitive;

%% convert to standard primitive problem
standard        = StandardPrimitiveProblem(problem);

%% Solve using DAMP
[R_est,t_est,info] = DAMP(standard,...
    'Damping',1,'TimeStep',0.3,'StopTh',1e-5,'SDamping',0.0,...
    'MaxIters',1000);
R_err              = getAngularError(standard.R, R_est);
t_err              = getTranslationError(standard.t, t_est);

fprintf('DAMP: R_err: %g[deg], t_err: %g[m], time: %g[s].\n',...
    R_err,t_err,info.time);

%% Plot
saveVideo = false;
fileName = sprintf('/Users/hankyang/Documents/MATLAB/DAMP/Videos/sample_PR.mp4');
PlotPrimitiveRegistration(problem,info.XItstTraj,info.YItstTraj,info.StateTraj,...
    'SaveVideo',saveVideo,...
    'FileName',fileName);





