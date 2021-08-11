function model = ComputePASCALAeroplaneClass(varargin)
params = inputParser;
params.CaseSensitive = false;

params.addParameter('Conf',0.5,...
    @(x) isscalar(x));

params.parse(varargin{:});

conf = params.Results.Conf;

S = load('/Users/hankyang/Documents/MATLAB/DAMP/data/aeroplane.mat','aeroplane');

car = S.aeroplane;

car_idx = 1:8;

num_cars = length(car_idx);
num_kpts = 8;

kpts_order = {'left_elevator',...
    'left_wing',...
    'noselanding',...
    'right_elevator',...
    'right_wing',...
    'rudder_lower',...
    'rudder_upper',...
    'tail',...
    };

cars = zeros(3,num_kpts,num_cars);
clouds = {};
for i = 1:num_cars
    ii = car_idx(i);
    clouds{end+1} = car(ii).vertices';
end

for i = 1:num_cars
    ii = car_idx(i);
    for j = 1:num_kpts
        pt = getfield(car(ii),kpts_order{j});
        if isempty(pt)
            error('no field %s.',kpts_order{j});
            pt = nan(3,1);
        end
        cars(:,j,i) = pt;
    end
end

cadmodels = zeros(3*num_cars,num_kpts);
for i = 1:num_cars
    cadmodels(blkIndices(i,3),:) = cars(:,:,i);
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
model.NumModels = num_models;
model.mu = mu;
model.C = C;
model.conf = conf;
model.ksq = ksq;
model.omega = omega;
model.clouds = clouds;
end