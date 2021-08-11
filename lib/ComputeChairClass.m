function model = ComputeChairClass(varargin)
params = inputParser;
params.CaseSensitive = false;

params.addParameter('Conf',0.6,...
    @(x) isscalar(x));

params.parse(varargin{:});

conf = params.Results.Conf;

S = load('/Users/hankyang/Documents/MATLAB/DAMP/data/chair.mat','chair');

chair = S.chair;

chair_idx = [1,3:9];
% chair_idx = 1:10;

num_chairs = length(chair_idx);
num_kpts = 10;

kpts_order = {'back_upper_left',...
    'back_upper_right',...
    'seat_upper_left',...
    'seat_upper_right',...
    'seat_lower_left',...
    'seat_lower_right',...
    'leg_upper_left',...
    'leg_upper_right',...
    'leg_lower_left',...
    'leg_lower_right'};

chairs = zeros(3,num_kpts,num_chairs);
clouds = {};
for i = 1:num_chairs
    ii = chair_idx(i);
    clouds{end+1} = chair(ii).vertices';
end

for i = 1:num_chairs
    ii = chair_idx(i);
    for j = 1:num_kpts
        pt = getfield(chair(ii),kpts_order{j});
        if isempty(pt)
            pt = nan(3,1);
        end
        chairs(:,j,i) = pt;
    end
end

cadmodels = zeros(3*num_chairs,num_kpts);
for i = 1:num_chairs
    cadmodels(blkIndices(i,3),:) = chairs(:,:,i);
end

num_models = size(cadmodels,1)/3;
num_kpts = size(cadmodels,2);

cars = {};
for i = 1:num_models
    cari = cadmodels(blkIndices(i,3),:);
    cari = cari - mean(cari,2);
    cars{end+1} = cari;
    cadmodels(blkIndices(i,3),:) = cari;
end

% for each kpt, compute its mean location and covariance matrix
C = zeros(3,3,num_kpts);
mu = zeros(3,num_kpts);

for i = 1:num_kpts
    kpti = zeros(3,num_models);
    for j = 1:num_models
        kpti(:,j) = cars{j}(:,i);
    end
    mu(:,i) = nanmean(kpti,2);
    C(:,:,i) = nancov((kpti - mu(:,i))');  
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

model.cadmodels = cadmodels;
model.NumKpts = num_kpts;
model.NumModels = num_chairs;
model.mu = mu;
model.C = C;
model.edges = chairedges;
model.conf = conf;
model.ksq = ksq;
model.omega = omega;
model.clouds = clouds;
end

function edges = chairedges()
% kpts_order = {'back_upper_left',... 1
%     'back_upper_right',... 2
%     'seat_upper_left',... 3
%     'seat_upper_right',... 4
%     'seat_lower_left',... 5
%     'seat_lower_right',... 6
%     'leg_upper_left',... 7
%     'leg_upper_right',... 8
%     'leg_lower_left',... 9
%     'leg_lower_right'}; 10
edges = [1,2;
         1,3;
         2,4;
         3,5;
         3,4;
         4,6;
         5,6;
         3,7;
         4,8;
         5,9;
         6,10];
end