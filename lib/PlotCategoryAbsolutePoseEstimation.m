function PlotCategoryAbsolutePoseEstimation(problem,XItstTraj,YItstTraj,StateTraj,varargin)
params = inputParser;
params.CaseSensitive = false;
params.addParameter('SaveVideo',false,...
    @(x) islogical(x));
params.addParameter('FileName','sample',...
    @(x) ischar(x));

params.parse(varargin{:});

SaveVideo = params.Results.SaveVideo;
FileName = params.Results.FileName;

global POINT_SIZE LINE_WIDTH

POINT_SIZE = 6;
LINE_WIDTH = 1.5;

SIZE = 5;

model = problem.model;
mu0 = model.mu;
C0 = model.C;

fig = figure;
set(fig, 'color','w', 'position', [100,100,800,800]);

hold on

num_steps = length(StateTraj);

F = {};

fprintf('Render step ')

for i = 1:num_steps
    
    fprintf('%d ',i);
    
    clf(fig);

    X = XItstTraj{i};
    Y = YItstTraj{i};
    s = StateTraj{i};
    
    t = s(1:3);
    R = Quat2Rotm(s(4:7));
    
    mu = R*mu0 + t;
    for j = 1:model.NumKpts
        C(:,:,j) = R*C0(:,:,j)*R';
    end
    model.mu = mu;
    model.C = C;
    
    PlotCamera(problem);
    hold on
    PlotModel(model)
    hold on
    
    
    h1 = plot3(X(1,:),X(2,:),X(3,:),'^',...
       'MarkerFaceColor','cyan','MarkerEdgeColor','cyan','MarkerSize',POINT_SIZE/2);
    
    h2 = plot3(Y(1,:),Y(2,:),Y(3,:),'^',...
            'MarkerFaceColor','cyan','MarkerEdgeColor','cyan','MarkerSize',3); 
    h = [h1,h2];
    
    for j = 1:size(X,2)
        xj = X(:,j);
        yj = Y(:,j);
        l = line([xj(1),yj(1)],[xj(2),yj(2)],[xj(3),yj(3)],...
            'Color','magenta','LineWidth',1.0);
        h(end+1) = l;
    end
    
    xlim([-SIZE,SIZE])
    ylim([-SIZE,SIZE])
    zlim([-0.5*SIZE,1.5*SIZE])

    view(-120,-20);
    
    if SaveVideo
        F{end+1}=getframe(gcf);
    end

    pause(0.01)
    
   
end

fprintf('\n');

if SaveVideo
    writerObj = VideoWriter(FileName,'MPEG-4');
    writerObj.FrameRate = 20;
    open(writerObj);
    % write the frames to the video
    nrImages = length(F);
    for i=1:nrImages
        frame = F{i}.cdata ;    
        writeVideo(writerObj,frame);
    end
    close(writerObj);
end

end

function PlotCamera(problem)

global POINT_SIZE LINE_WIDTH

N = problem.N;
tform = rigid3d(eye(3),zeros(1,3));
cam = plotCamera('AbsolutePose',tform,'Size',0.2,'Color','green','Opacity',0.1);

hold on

PLANE_SIZE = 1;
x_center = [0;0;1];
v1 = [1;0;0];
v2 = [0;1;0];

x_bl = x_center - PLANE_SIZE*v1 - PLANE_SIZE*v2;
x_br = x_center + PLANE_SIZE*v1 - PLANE_SIZE*v2;
x_tr = x_center + PLANE_SIZE*v1 + PLANE_SIZE*v2;
x_tl = x_center - PLANE_SIZE*v1 + PLANE_SIZE*v2;
corners = [x_bl,x_br,x_tr,x_tl];
patch(corners(1,:),corners(2,:),corners(3,:),'red',...
    'FaceColor','red','FaceAlpha',0.2,'EdgeColor','none');
hold on

x = problem.x;
LINE_LENGTH = 10.0;
for i = 1:N
    xi = x(:,i);
    plot3(xi(1),xi(2),1.0,'o',...
        'MarkerFaceColor','red',...
        'MarkerEdgeColor','none',...
        'MarkerSize',POINT_SIZE/2);
    line([0,LINE_LENGTH*xi(1)],[0,LINE_LENGTH*xi(2)],[0,LINE_LENGTH*1.0],...
        'Color',[1,0,0,0.2],'LineWidth',0.5);
    hold on
end
axis equal
axis off
end


function PlotModel(model)

global POINT_SIZE LINE_WIDTH

conf = model.conf;
mu = model.mu;
C = model.C;
edges = model.edges;
num_kpts = size(mu,2);

for i = 1:num_kpts
    h = error_ellipse(C(:,:,i),mu(:,i),'conf',conf);
    hold on
end

hold on

num_edges = size(edges,1);

for i = 1:num_edges
    one = edges(i,1);
    two = edges(i,2);
    xone = mu(:,one);
    xtwo = mu(:,two);
    
    x = [xone,xtwo];
    
    line(x(1,:),x(2,:),x(3,:),'Color',[1,0,0,0.5],'LineWidth',1.0);
    hold on
end

axis equal
axis off

end
