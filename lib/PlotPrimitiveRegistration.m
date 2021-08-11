function PlotPrimitiveRegistration(problem,XItstTraj,YItstTraj,StateTraj,varargin)
params = inputParser;
params.CaseSensitive = false;
params.addParameter('SaveVideo',false,...
    @(x) islogical(x));
params.addParameter('FileName','sample',...
    @(x) ischar(x));

params.parse(varargin{:});

SaveVideo = params.Results.SaveVideo;
FileName = params.Results.FileName;

global POINT_SIZE LINE_WIDTH CLOUD_SIZE CLOUD_ALPHA FACEALPHA EDGEALPHA

CLOUD_SIZE = 9;
POINT_SIZE = 6;
LINE_WIDTH = 2;
CLOUD_ALPHA = 0.2;
FACEALPHA = 0.3;
EDGEALPHA = 0.3;

SIZE = 20;

cloud = problem.cloud;
scene = problem.scene;
scene_center = mean(scene,2);

fig = figure;
set(fig, 'color','w', 'position', [100,100,800,800]);

PlotModel(problem.PrimitiveSurfs);
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
    
    scenecloudi = R*(cloud-scene_center) + t;
    
    h0 = scatter3(scenecloudi(1,:),scenecloudi(2,:),scenecloudi(3,:),CLOUD_SIZE,...
        'o',...
        'MarkerFaceColor','blue','MarkerEdgeColor','blue',...
        'MarkerFaceAlpha',CLOUD_ALPHA,'MarkerEdgeAlpha',CLOUD_ALPHA);
    
    h1 = plot3(X(1,:),X(2,:),X(3,:),'o',...
       'MarkerFaceColor','blue','MarkerEdgeColor','blue','MarkerSize',POINT_SIZE);
    
    h2 = plot3(Y(1,:),Y(2,:),Y(3,:),'^',...
            'MarkerFaceColor','cyan','MarkerEdgeColor','cyan','MarkerSize',3); 
    h = [h0,h1,h2];
    
    for j = 1:size(Y,2)
        xj = X(:,j);
        yj = Y(:,j);
        l = line([xj(1),yj(1)],[xj(2),yj(2)],[xj(3),yj(3)],...
            'Color','magenta','LineWidth',LINE_WIDTH);
        h(end+1) = l;
    end
    
    axis off
    
    xlim([-SIZE,SIZE])
    ylim([-SIZE,SIZE])
    zlim([-SIZE,SIZE])
    
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

function PlotModel(model)
global FACEALPHA EDGEALPHA

for i = 1:length(model)
    data = model{i}.surf;
    s = surf(data{1},data{2},data{3});
    s.FaceColor = 'red';
    s.FaceAlpha = FACEALPHA;
    s.EdgeAlpha = EDGEALPHA;
    hold on
end

axis equal

end
