function PlotSPEED(img,keypoints,Kinv,varargin)

params = inputParser;
params.CaseSensitive = false;
params.addParameter('LineLength',4,...
    @(x) isscalar(x));
params.addParameter('LineAlpha',0.2,...
    @(x) isscalar(x));
params.addParameter('LineWidth',1.0,...
    @(x) isscalar(x));

params.parse(varargin{:});

LineLength = params.Results.LineLength;
LineAlpha = params.Results.LineAlpha;
LineWidth = params.Results.LineWidth;

if length(size(img)) > 2
    [height,width,~] = size(img);
    rgbimg = double(img) ./ 255;
else
    [height,width] = size(img);
    rgbimg = cat(3,img,img,img);
end

corners_pixel = [0,0,1;
                 width,0,1;
                 width,height,1;
                 0,height,1];
corners_meter = Kinv * corners_pixel';
x_min = min(corners_meter(1,:));
x_max = max(corners_meter(1,:));
y_min = min(corners_meter(2,:));
y_max = max(corners_meter(2,:));

[X,Y] = meshgrid(linspace(x_min,x_max,width),...
                 linspace(y_min,y_max,height));

Z = ones(size(X));

% Plot image
h = surface(X,Y,Z,rgbimg,'facecolor','texturemap','edgecolor','none',...
    'CDataMapping','direct');

hold on

% spawn a camera
tform = rigid3d(eye(3),zeros(1,3));
cam = plotCamera('AbsolutePose',tform,'Size',0.05,'Color','green','Opacity',0.1);

hold on
% plot keypoints and bearing vectors
keypoints_meter = Kinv*[keypoints;ones(1,size(keypoints,2))];

num_keypoints = size(keypoints_meter,2);

LINE_LENGTH = LineLength;

for i = 1:num_keypoints
    xi = keypoints_meter(:,i);
    plot3(xi(1),xi(2),xi(3),'square',...
        'MarkerFaceColor','yellow',...
        'MarkerEdgeColor','none',...
        'MarkerSize',6);
    line([0,LINE_LENGTH*xi(1)],[0,LINE_LENGTH*xi(2)],[0,LINE_LENGTH*xi(3)],...
        'Color',[1,0,0,LineAlpha],'LineWidth',LineWidth);
    hold on
end

% axis equal
view(30,30);
end