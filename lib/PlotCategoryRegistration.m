function PlotCategoryRegistration(problem,XItstTraj,YItstTraj,StateTraj,varargin)
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
POINT_SIZE = 8;
LINE_WIDTH = 1.5;
CLOUD_ALPHA = 0.2;

scenecloud = problem.scenecloud;
scene = problem.scene;
scene_center = mean(scene,2);

switch problem.class
    case 'car'
        SIZE = 3;
    case 'human'
        SIZE = 4;
    case 'chair'
        SIZE = 1;
end

fig = figure;
set(fig, 'color','w', 'position', [100,100,800,800]);

PlotModel(problem.model);
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
    
    scenecloudi = R*(scenecloud-scene_center) + t;
    
    h0 = scatter3(scenecloudi(1,:),scenecloudi(2,:),scenecloudi(3,:),CLOUD_SIZE,...
        'o',...
        'MarkerFaceColor','blue','MarkerEdgeColor','blue',...
        'MarkerFaceAlpha',CLOUD_ALPHA,'MarkerEdgeAlpha',CLOUD_ALPHA);
    
    h1 = plot3(X(1,:),X(2,:),X(3,:),'o',...
       'MarkerFaceColor','blue','MarkerEdgeColor','blue','MarkerSize',POINT_SIZE);
    
    h2 = plot3(Y(1,1:5),Y(2,1:5),Y(3,1:5),'^',...
            'MarkerFaceColor','cyan','MarkerEdgeColor','cyan','MarkerSize',3); 
    h = [h0,h1,h2];
    
%     if problem.class == "human"
%         edges = problem.model.edges;
%         colors = problem.model.edgeColors;
%         num_edges = size(edges,1);
%         for j = 1:num_edges
%             one = edges(j,1);
%             two = edges(j,2);
%             xone = X(:,one);
%             xtwo = X(:,two);
%             x = [xone,xtwo];
% 
%             l = line(x(1,:),x(2,:),x(3,:),'Color',colors{j},'LineWidth',2);
%             h = [h,l];
%             hold on
%         end 
%     elseif problem.class == "chair"
%         edges = problem.model.edges;
%         num_edges = size(edges,1);
%         for j = 1:num_edges
%             one = edges(j,1);
%             two = edges(j,2);
%             xone = X(:,one);
%             xtwo = X(:,two);
%             x = [xone,xtwo];
% 
%             l = line(x(1,:),x(2,:),x(3,:),'Color',[0,0,1],'LineWidth',2);
%             h = [h,l];
%             hold on
%         end 
%     end
    
    
    
    for j = 1:5
        xj = X(:,j);
        yj = Y(:,j);
        l = line([xj(1),yj(1)],[xj(2),yj(2)],[xj(3),yj(3)],...
            'Color','magenta','LineWidth',2.0);
        h(end+1) = l;
    end
    
    switch problem.class
        case 'car'
            view(8,46);
        case 'human'
            view(-130,0);
        case 'chair'
            view(8,46);
    end
    
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
    
    line(x(1,:),x(2,:),x(3,:),'Color',[1,0,0,0.5],'LineWidth',2.0);
    hold on
end

axis equal
axis off

end
