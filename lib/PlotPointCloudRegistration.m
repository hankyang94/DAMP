function PlotPointCloudRegistration(problem,XItstTraj,YItstTraj,StateTraj,varargin)
params = inputParser;
params.CaseSensitive = false;
params.addParameter('SaveVideo',false,...
    @(x) islogical(x));
params.addParameter('FileName','sample',...
    @(x) ischar(x));


params.parse(varargin{:});

SaveVideo = params.Results.SaveVideo;
FileName = params.Results.FileName;

global POINT_SIZE LINE_WIDTH CLOUD_SIZE CLOUD_ALPHA

CLOUD_SIZE = 8;
POINT_SIZE = 6;
LINE_WIDTH = 1.5;
CLOUD_ALPHA = 0.2;

SIZE = 0.3;

fig = figure;
set(fig, 'color','w', 'position', [100,100,800,800]);
model = problem.model;
model_center = mean(problem.X,2);
scene_center = mean(problem.scene,2);

PlotModel(problem.scene,problem.Y);
hold on

num_steps = length(StateTraj);

F = {};
h = [];

for i = 1:num_steps
%     clf(fig);
    
    delete(h);
    
    X = XItstTraj{i};
    Y = YItstTraj{i};
    s = StateTraj{i};
    t = s(1:3);
    q = s(4:7);
    R = Quat2Rotm(q);
    modeli = R*(model-model_center) + t;
    
    h0 = scatter3(modeli(1,:),modeli(2,:),modeli(3,:),CLOUD_SIZE,...
        'o',...
        'MarkerFaceColor','blue','MarkerEdgeColor','blue',...
        'MarkerFaceAlpha',CLOUD_ALPHA,'MarkerEdgeAlpha',CLOUD_ALPHA);
    
    h1 = plot3(X(1,:),X(2,:),X(3,:),'o',...
       'MarkerFaceColor','blue','MarkerEdgeColor','blue','MarkerSize',POINT_SIZE);
    
    h2 = plot3(Y(1,1:5),Y(2,1:5),Y(3,1:5),'^',...
            'MarkerFaceColor','cyan','MarkerEdgeColor','cyan','MarkerSize',3); 
    h = [h0,h1,h2]; 
    
    for j = 1:5
        xj = X(:,j);
        yj = Y(:,j);
        l = line([xj(1),yj(1)],[xj(2),yj(2)],[xj(3),yj(3)],...
            'Color','magenta','LineWidth',1.0);
        h(end+1) = l;
    end
    
    
    view(-30,64);
    axis equal
    axis off
    
    xlim(scene_center(1)+[-SIZE,SIZE])
    ylim(scene_center(2)+[-SIZE,SIZE])
    zlim(scene_center(3)+[-SIZE,SIZE])
    
    if SaveVideo
        F{end+1}=getframe(gcf);
    end

    pause(0.01)
end

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

function PlotModel(model,Y)
global CLOUD_SIZE 
global POINT_SIZE
global CLOUD_ALPHA

scatter3(model(1,:),model(2,:),model(3,:),CLOUD_SIZE,...
    'o',...
    'MarkerFaceColor','r','MarkerEdgeColor','r',...
    'MarkerFaceAlpha',CLOUD_ALPHA,'MarkerEdgeAlpha',CLOUD_ALPHA)

hold on
plot3(Y(1,:),Y(2,:),Y(3,:),'o',...
    'MarkerFaceColor','r','MarkerEdgeColor','r',...
    'MarkerSize',POINT_SIZE)

hold on
axis equal

end
