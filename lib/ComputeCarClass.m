function model = ComputeCarClass(varargin)
params = inputParser;
params.CaseSensitive = false;

params.addParameter('Conf',0.6,...
    @(x) isscalar(x));

params.parse(varargin{:});

conf = params.Results.Conf;

S = load('/Users/hankyang/Documents/MATLAB/DAMP/data/cadmodels.mat','cadmodels','edge');

cadmodels = S.cadmodels;
edges = S.edge;

num_models = size(cadmodels,1)/3;
num_kpts = size(cadmodels,2);

cars = {};
for i = 1:num_models
    cari = cadmodels(blkIndices(i,3),:);
    cari = cari - mean(cari,2);
    cars{end+1} = cari;
end

% for each kpt, compute its mean location and covariance matrix
C = zeros(3,3,num_kpts);
mu = zeros(3,num_kpts);

for i = 1:num_kpts
    kpti = zeros(3,num_models);
    for j = 1:num_models
        kpti(:,j) = cars{j}(:,i);
    end
    mu(:,i) = mean(kpti,2);
    C(:,:,i) = cov((kpti - mu(:,i))');  
end

% Some covariance matrices are ill-conditioned, manually adjust them
% For some of these keypoints, maybe better treat them as planes
for i = 1:num_kpts
    Ci = C(:,:,i);
    [V,D] = eig(Ci);
    d = diag(D);
    [~,I] = sort(d,'descend');
    V = V(:,I);
    D = D(I,I);
    d = diag(D);
    kappa = d(1)/d(end); % condition number
    if kappa > 1e3
        d(3) = d(1)/1e3;
        D = diag(d);
        C(:,:,i) = V*D*V';
    end
end

ksq = chi2inv(conf,3);
omega = zeros(3,3,num_kpts);
for i = 1:num_kpts
    omega(:,:,i) = inv(C(:,:,i))/ksq;
end
model.NumModels = num_models;
model.cadmodels = cadmodels;
model.NumKpts = num_kpts;
model.mu = mu;
model.C = C;
model.edges = edges;
model.conf = conf;
model.ksq = ksq;
model.omega = omega;
end