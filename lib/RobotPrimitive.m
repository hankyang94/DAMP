function problem = RobotPrimitive(varargin)
params = inputParser;
params.CaseSensitive = false;

params.addParameter('NoiseSigma',0.1,...
    @(x) isscalar(x));
params.addParameter('PlotRobot',true,...
    @(x) islogical(x));

params.parse(varargin{:});

NoiseSigma = params.Results.NoiseSigma;
PlotRobot = params.Results.PlotRobot;


% Generate and visualize a robot primitive
FACEALPHA = 0.3;
EDGEALPHA = 0.3;

% A sphere (the robot's head)
S.type = 'sphere';
S.x = [0;0;4];
S.r = 4;

% Six planes to form a cube
H1.type = 'plane';
H1.x = [0;0;0];
H1.n = [0;0;1];
H2.type = 'plane';
H2.x = [0;0;-15];
H2.n = [0;0;-1];
H3.type = 'plane';
H3.x = [-5;0;-7.5];
H3.n = [-1;0;0];
H4.type = 'plane';
H4.x = [5;0;-7.5];
H4.n = [1;0;0];
H5.type = 'plane';
H5.x = [0;5;-7.5];
H5.n = [0;1;0];
H6.type = 'plane';
H6.x = [0;-5;-7.5];
H6.n = [0;-1;0];

% Two cylinders to form legs
C1.type = 'cylinder';
C1.x = [-2;0;-15];
C1.v = [0;0;-1];
C1.r = 1;
C2.type = 'cylinder';
C2.x = [2;0;-15];
C2.v = [0;0;-1];
C2.r = 1;

% Two cylinders to form arms
C3.type = 'cylinder';
C3.x = [5;0;-5];
C3.v = [1;0;0];
C3.r = 1.0;
C4.type = 'cylinder';
C4.x = [-5;0;-5];
C4.v = [-1;0;0];
C4.r = 1.0;

% Two cones for ears
K1.type = 'cone';
K1.x = [4;0;4];
K1.v = [1;0;0];
K1.theta = atan(0.5);
K2.type = 'cone';
K2.x = [-4;0;4];
K2.v = [-1;0;0];
K2.theta = atan(0.5);

[X,Y,Z] = sphere;
XS = X*S.r + S.x(1);
YS = Y*S.r + S.x(2);
ZS = Z*S.r + S.x(3);
S_surf = surf(XS,YS,ZS);
S_surf.FaceColor = 'red';
S_surf.FaceAlpha = FACEALPHA;
S_surf.EdgeAlpha = EDGEALPHA;
hold on
[X1,Y1,Z1] = meshgrid(-5:0.5:5,-5:0.5:5,0:0);
H1_surf = surf(X1,Y1,Z1);
H1_surf.FaceColor = 'red';
H1_surf.FaceAlpha = FACEALPHA;
H1_surf.EdgeAlpha = EDGEALPHA;
hold on
[X2,Y2,Z2] = meshgrid(-5:0.5:5,-5:0.5:5,-15:-15);
H2_surf = surf(X2,Y2,Z2);
H2_surf.FaceColor = 'red';
H2_surf.FaceAlpha = FACEALPHA;
H2_surf.EdgeAlpha = EDGEALPHA;
hold on
[Y3,Z3,X3] = meshgrid(-5:0.5:5,-15:0.5:0,-5:-5);
H3_surf = surf(X3,Y3,Z3);
H3_surf.FaceColor = 'red';
H3_surf.FaceAlpha = FACEALPHA;
H3_surf.EdgeAlpha = EDGEALPHA;
hold on
[Y4,Z4,X4] = meshgrid(-5:0.5:5,-15:0.5:0,5:5);
H4_surf = surf(X4,Y4,Z4);
H4_surf.FaceColor = 'red';
H4_surf.FaceAlpha = FACEALPHA;
H4_surf.EdgeAlpha = EDGEALPHA;
hold on
[X5,Z5,Y5] = meshgrid(-5:0.5:5,-15:0.5:0,5:5);
H5_surf = surf(X5,Y5,Z5);
H5_surf.FaceColor = 'red';
H5_surf.FaceAlpha = FACEALPHA;
H5_surf.EdgeAlpha = EDGEALPHA;
hold on
[X6,Z6,Y6] = meshgrid(-5:0.5:5,-15:0.5:0,-5:-5);
H6_surf = surf(X6,Y6,Z6);
H6_surf.FaceColor = 'red';
H6_surf.FaceAlpha = FACEALPHA;
H6_surf.EdgeAlpha = EDGEALPHA;
hold on
[XC1,YC1,ZC1] = cylinder;
ZC1 = -5*ZC1;
XC1 = XC1 + C1.x(1);
YC1 = YC1 + C1.x(2);
ZC1 = ZC1 + C1.x(3);
C1_surf = surf(XC1,YC1,ZC1);
C1_surf.FaceColor = 'red';
C1_surf.FaceAlpha = FACEALPHA;
C1_surf.EdgeAlpha = EDGEALPHA;
hold on
[XC2,YC2,ZC2] = cylinder;
ZC2 = -5*ZC2;
XC2 = XC2 + C2.x(1);
YC2 = YC2 + C2.x(2);
ZC2 = ZC2 + C2.x(3);
C2_surf = surf(XC2,YC2,ZC2);
C2_surf.FaceColor = 'red';
C2_surf.FaceAlpha = FACEALPHA;
C2_surf.EdgeAlpha = EDGEALPHA;
hold on

[ZC3,YC3,XC3] = cylinder;
XC3 = 5*XC3;
XC3 = XC3 + C3.x(1);
YC3 = YC3 + C3.x(2);
ZC3 = ZC3 + C3.x(3);
C3_surf = surf(XC3,YC3,ZC3);
C3_surf.FaceColor = 'red';
C3_surf.FaceAlpha = FACEALPHA;
C3_surf.EdgeAlpha = EDGEALPHA;
hold on

[ZC4,YC4,XC4] = cylinder;
XC4 = -5*XC4;
XC4 = XC4 + C4.x(1);
YC4 = YC4 + C4.x(2);
ZC4 = ZC4 + C4.x(3);
C4_surf = surf(XC4,YC4,ZC4);
C4_surf.FaceColor = 'red';
C4_surf.FaceAlpha = FACEALPHA;
C4_surf.EdgeAlpha = EDGEALPHA;
hold on

r = linspace(0,2,10);
th = linspace(0,2*pi,20);
[R,T] = meshgrid(r,th);
YK1 = R.*cos(T);
ZK1 = R.*sin(T);
XK1 = 2*R;
XK1 = XK1 + K1.x(1);
YK1 = YK1 + K1.x(2);
ZK1 = ZK1 + K1.x(3);
K1_surf = surf(XK1,YK1,ZK1);
K1_surf.FaceColor = 'red';
K1_surf.FaceAlpha = FACEALPHA;
K1_surf.EdgeAlpha = EDGEALPHA;
hold on

YK2 = R.*cos(T);
ZK2 = R.*sin(T);
XK2 = -2*R;
XK2 = XK2 + K2.x(1);
YK2 = YK2 + K2.x(2);
ZK2 = ZK2 + K2.x(3);
K2_surf = surf(XK2,YK2,ZK2);
K2_surf.FaceColor = 'red';
K2_surf.FaceAlpha = FACEALPHA;
K2_surf.EdgeAlpha = EDGEALPHA;
axis equal



PrimitiveModel = {S,H1,H2,H3,H4,H5,H6,C1,C2,C3,C4,K1,K2};
S_surf = S; S_surf.surf = {XS,YS,ZS};
H1_surf = H1; H1_surf.surf = {X1,Y1,Z1};
H2_surf = H2; H2_surf.surf = {X2,Y2,Z2};
H3_surf = H3; H3_surf.surf = {X3,Y3,Z3};
H4_surf = H4; H4_surf.surf = {X4,Y4,Z4};
H5_surf = H5; H5_surf.surf = {X5,Y5,Z5};
H6_surf = H6; H6_surf.surf = {X6,Y6,Z6};
C1_surf = C1; C1_surf.surf = {XC1,YC1,ZC1};
C2_surf = C2; C2_surf.surf = {XC2,YC2,ZC2};
C3_surf = C3; C3_surf.surf = {XC3,YC3,ZC3};
C4_surf = C4; C4_surf.surf = {XC4,YC4,ZC4};
K1_surf = K1; K1_surf.surf = {XK1,YK1,ZK1};
K2_surf = K2; K2_surf.surf = {XK2,YK2,ZK2};
PrimitiveSurfs = {S_surf,H1_surf,H2_surf,H3_surf,H4_surf,H5_surf,H6_surf,...
    C1_surf,C2_surf,C3_surf,C4_surf,K1_surf,K2_surf};

problem.PrimitiveModel = PrimitiveModel;
problem.PrimitiveSurfs = PrimitiveSurfs;

% Random sample points and generate correspondences
% sample the sphere
S_ns = 20;
S_samples = randn(3,S_ns);
S_samples = S.r * S_samples ./ (sum(S_samples.^2,1).^(0.5));
S_samples = S_samples + S.x;
% cloud for visualization
S_ns_cloud = 400;
S_cloud_samples = randn(3,S_ns_cloud);
S_cloud_samples = S.r * S_cloud_samples ./ (sum(S_cloud_samples.^2,1).^(0.5));
S_cloud_samples = S_cloud_samples + S.x;

% sample the first plane
H1_ns = 10;
H1_samples = rand(2,H1_ns);
H1_samples = H1_samples .* [10;10] + [-5;-5];
H1_samples = [H1_samples;zeros(1,H1_ns)];
% cloud for vis
H1_ns_cloud = 200;
H1_cloud_samples = rand(2,H1_ns_cloud);
H1_cloud_samples = H1_cloud_samples .* [10;10] + [-5;-5];
H1_cloud_samples = [H1_cloud_samples;zeros(1,H1_ns_cloud)];


% sample the second plane
H2_ns = 10;
H2_samples = rand(2,H2_ns);
H2_samples = H2_samples .* [10;10] + [-5;-5];
H2_samples = [H2_samples;-15*ones(1,H2_ns)];
% cloud for vis
H2_ns_cloud = 200;
H2_cloud_samples = rand(2,H2_ns_cloud);
H2_cloud_samples = H2_cloud_samples .* [10;10] + [-5;-5];
H2_cloud_samples = [H2_cloud_samples;-15*ones(1,H2_ns_cloud)];

% sample the third plane
H3_ns = 10;
H3_samples = rand(2,H3_ns);
H3_samples = H3_samples .* [10;15] + [-5;-7.5];
H3_samples = [zeros(1,H3_ns);H3_samples];
H3_samples = H3_samples + H3.x;
% cloud for vis
H3_ns_cloud = 200;
H3_cloud_samples = rand(2,H3_ns_cloud);
H3_cloud_samples = H3_cloud_samples .* [10;15] + [-5;-7.5];
H3_cloud_samples = [zeros(1,H3_ns_cloud);H3_cloud_samples];
H3_cloud_samples = H3_cloud_samples + H3.x;

% sample the fourth plane
H4_ns = 10;
H4_samples = rand(2,H4_ns);
H4_samples = H4_samples .* [10;15] + [-5;-7.5];
H4_samples = [zeros(1,H4_ns);H4_samples];
H4_samples = H4_samples + H4.x;
% cloud for vis
H4_ns_cloud = 200;
H4_cloud_samples = rand(2,H4_ns_cloud);
H4_cloud_samples = H4_cloud_samples .* [10;15] + [-5;-7.5];
H4_cloud_samples = [zeros(1,H4_ns_cloud);H4_cloud_samples];
H4_cloud_samples = H4_cloud_samples + H4.x;

% sample the fifth plane
H5_ns = 5;
H5_samples = rand(2,H5_ns);
H5_samples = H5_samples .* [10;15] + [-5;-7.5];
H5_samples = [H5_samples(1,:);zeros(1,H5_ns);H5_samples(2,:)];
H5_samples = H5_samples + H5.x;
% cloud for vis
H5_ns_cloud = 200;
H5_cloud_samples = rand(2,H5_ns_cloud);
H5_cloud_samples = H5_cloud_samples .* [10;15] + [-5;-7.5];
H5_cloud_samples = [H5_cloud_samples(1,:);zeros(1,H5_ns_cloud);H5_cloud_samples(2,:)];
H5_cloud_samples = H5_cloud_samples + H5.x;

% sample the sixth plane
H6_ns = 5;
H6_samples = rand(2,H6_ns);
H6_samples = H6_samples .* [10;15] + [-5;-7.5];
H6_samples = [H6_samples(1,:);zeros(1,H6_ns);H6_samples(2,:)];
H6_samples = H6_samples + H6.x;
% cloud for vis
H6_ns_cloud = 200;
H6_cloud_samples = rand(2,H6_ns_cloud);
H6_cloud_samples = H6_cloud_samples .* [10;15] + [-5;-7.5];
H6_cloud_samples = [H6_cloud_samples(1,:);zeros(1,H6_ns_cloud);H6_cloud_samples(2,:)];
H6_cloud_samples = H6_cloud_samples + H6.x;

% sample C1
C1_ns = 5;
th = rand(1,C1_ns) * 2*pi;
C1_samples = [cos(th);sin(th);-5*rand(1,C1_ns)];
C1_samples = C1_samples + C1.x;
% cloud for vis
C1_ns_cloud = 100;
th = rand(1,C1_ns_cloud) * 2*pi;
C1_cloud_samples = [cos(th);sin(th);-5*rand(1,C1_ns_cloud)];
C1_cloud_samples = C1_cloud_samples + C1.x;

% sample C2
C2_ns = 5;
th = rand(1,C2_ns) * 2*pi;
C2_samples = [cos(th);sin(th);-5*rand(1,C2_ns)];
C2_samples = C2_samples + C2.x;
% cloud for vis
C2_ns_cloud = 100;
th = rand(1,C2_ns_cloud) * 2*pi;
C2_cloud_samples = [cos(th);sin(th);-5*rand(1,C2_ns_cloud)];
C2_cloud_samples = C2_cloud_samples + C2.x;

% sample C3
C3_ns = 5;
th = rand(1,C3_ns) * 2*pi;
C3_samples = [5*rand(1,C3_ns);cos(th);sin(th)];
C3_samples = C3_samples + C3.x;
% cloud for vis
C3_ns_cloud = 100;
th = rand(1,C3_ns_cloud) * 2*pi;
C3_cloud_samples = [5*rand(1,C3_ns_cloud);cos(th);sin(th)];
C3_cloud_samples = C3_cloud_samples + C3.x;

% sample C4
C4_ns = 5;
th = rand(1,C4_ns) * 2*pi;
C4_samples = [-5*rand(1,C4_ns);cos(th);sin(th)];
C4_samples = C4_samples + C4.x;
% cloud for vis
C4_ns_cloud = 100;
th = rand(1,C4_ns_cloud) * 2*pi;
C4_cloud_samples = [-5*rand(1,C4_ns_cloud);cos(th);sin(th)];
C4_cloud_samples = C4_cloud_samples + C4.x;

% sample K1
K1_ns = 5;
th = rand(1,K1_ns) * 2*pi;
height = 4*rand(1,K1_ns);
K1_samples = [height;height*0.5.*cos(th);height*0.5.*sin(th)];
K1_samples = K1_samples + K1.x;
% cloud for vis
K1_ns_cloud = 200;
th = rand(1,K1_ns_cloud) * 2*pi;
height = 4*rand(1,K1_ns_cloud);
K1_cloud_samples = [height;height*0.5.*cos(th);height*0.5.*sin(th)];
K1_cloud_samples = K1_cloud_samples + K1.x;

% sample K2
K2_ns = 5;
th = rand(1,K2_ns) * 2*pi;
height = 4*rand(1,K2_ns);
K2_samples = [-height;height*0.5.*cos(th);height*0.5.*sin(th)];
K2_samples = K2_samples + K2.x;
% cloud for vis
K2_ns_cloud = 200;
th = rand(1,K2_ns_cloud) * 2*pi;
height = 4*rand(1,K2_ns_cloud);
K2_cloud_samples = [-height;height*0.5.*cos(th);height*0.5.*sin(th)];
K2_cloud_samples = K2_cloud_samples + K2.x;

% build up keypoint correspondences
scene = [S_samples(:,1),...
     H1_samples(:,1),...
     H3_samples(:,1),...
     H5_samples(:,1),...
     C1_samples(:,1),...
     K1_samples(:,1),...
     S_samples(:,2:end),...
     H1_samples(:,2:end),...
     H3_samples(:,2:end),...
     H5_samples(:,2:end),...
     H2_samples,...
     H4_samples,...
     H6_samples,...
     C1_samples(:,2:end),...
     C2_samples,...
     C3_samples,...
     C4_samples,...
     K1_samples(:,2:end),...
     K2_samples];
 
primitives = {S,H1,H3,H5,C1,K1};
for i = 2:S_ns
    primitives{end+1} = S;
end
for i = 2:H1_ns
    primitives{end+1} = H1;
end
for i = 2:H3_ns
    primitives{end+1} = H3;
end
for i = 2:H5_ns
    primitives{end+1} = H5;
end
for i = 1:H2_ns
    primitives{end+1} = H2;
end
for i = 1:H4_ns
    primitives{end+1} = H4;
end
for i = 1:H6_ns
    primitives{end+1} = H6;
end
for i = 2:C1_ns
    primitives{end+1} = C1;
end
for i = 1:C2_ns
    primitives{end+1} = C2;
end
for i = 1:C3_ns
    primitives{end+1} = C3;
end
for i = 1:C4_ns
    primitives{end+1} = C4;
end
for i = 2:K1_ns
    primitives{end+1} = K1;
end
for i = 1:K2_ns
    primitives{end+1} = K2;
end

assert(length(primitives)==size(scene,2),'number of points not match number of primitives.');

% build up the cloud
cloud = [S_cloud_samples,...
         H1_cloud_samples,...
         H2_cloud_samples,...
         H3_cloud_samples,...
         H4_cloud_samples,...
         H5_cloud_samples,...
         H6_cloud_samples,...
         C1_cloud_samples,...
         C2_cloud_samples,...
         C3_cloud_samples,...
         C4_cloud_samples,...
         K1_cloud_samples,...
         K2_cloud_samples];

R = rand_rotation;

R = rand_rotation;
t = randn(3,1);
t = t/norm(t);
t = 20*rand*t;
% NoiseSigma = NoiseSigma;
fprintf('Noise: %g.\n',NoiseSigma);
scene = R*scene + t + NoiseSigma*randn(3,length(primitives));
cloud = R*cloud + t + NoiseSigma*randn(3,size(cloud,2));

R_gt = R';
t_gt = -R'*t;

hold on
scatter3(scene(1,:),scene(2,:),scene(3,:),10,'blue','filled');
hold on
scatter3(cloud(1,:),cloud(2,:),cloud(3,:),4,'magenta','filled');

problem.N = length(primitives);
problem.primitives = primitives;
problem.scene = scene;
problem.R_gt = R_gt;
problem.t_gt = t_gt;
problem.cloud = cloud;
problem.type = 'PrimitiveRegistration';


% Plot the primitive surfaces
% figure;
% for i = 1:length(problem.PrimitiveSurfs)
%     data = problem.PrimitiveSurfs{i}.surf;
%     s = surf(data{1},data{2},data{3});
%     s.FaceColor = 'red';
%     s.FaceAlpha = FACEALPHA;
%     s.EdgeAlpha = EDGEALPHA;
%     hold on
% end
% axis equal

end