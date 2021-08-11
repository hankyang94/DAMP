function PlotSPEEDfromDAMP(img,keypoints,camera_Kinv,problem,XItstTraj,YItstTraj,StateTraj,varargin)
params = inputParser;
params.CaseSensitive = false;
params.addParameter('SaveVideo',false,...
    @(x) islogical(x));
params.addParameter('FileName','sample',...
    @(x) ischar(x));

params.parse(varargin{:});

SaveVideo = params.Results.SaveVideo;
FileName = params.Results.FileName;

load('./data/speed_model_3d_plot.mat');
pts = pts';

global POINT_SIZE LINE_WIDTH

POINT_SIZE = 6;
LINE_WIDTH = 1.5;

SIZE = 5;

fig = figure;
set(fig, 'color','w', 'position', [100,100,800,800]);

PlotSPEED(img,keypoints,camera_Kinv);
hold on

num_steps = length(StateTraj);

F = {};
h = [];
for i = num_steps:num_steps
%     clf(fig);
    
    delete(h);
    
    X = XItstTraj{i};
    Y = YItstTraj{i};
    
    h1 = PlotSPEEDWireFrame(StateTraj{i},pts,edges);
    
    temp = 1:size(Y,2);
    
    h2 = plot3(Y(1,temp),Y(2,temp),Y(3,temp),'^',...
            'MarkerFaceColor','cyan','MarkerEdgeColor','cyan','MarkerSize',POINT_SIZE); 
    
    h = [h1,h2];
    
    for j = 1:size(Y,2)
        xj = X(:,j);
        yj = Y(:,j);
        l = line([xj(1),yj(1)],[xj(2),yj(2)],[xj(3),yj(3)],...
            'Color','magenta','LineWidth',1.5);
        h = [h,l];
    end

    axis equal
    axis off
    
    xlim([-2,2])
    ylim([-2,2])
    zlim([-1,6])
    
    view(35,11);
    
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

function h = PlotSPEEDWireFrame(s,pts,edges)
global POINT_SIZE
pts_key = pts(:,1:11);
center = mean(pts_key,2);
pts = pts - center;

t = s(1:3);
R = Quat2Rotm(s(4:7));
pts_now = R*pts + t;

pts_key = pts_now(:,1:11);
pts_con = pts_now(:,12:end);

h1 = plot3(pts_key(1,:),pts_key(2,:),pts_key(3,:),'o',...
       'MarkerFaceColor','blue','MarkerEdgeColor','blue','MarkerSize',POINT_SIZE);
h2 = plot3(pts_con(1,:),pts_con(2,:),pts_con(3,:),'o',...
       'MarkerFaceColor','blue','MarkerEdgeColor','blue','MarkerSize',2);

h = [h1,h2];
nrEdges = size(edges,1);
for i = 1:nrEdges
    
    xj = pts_now(:,edges(i,1));
    yj = pts_now(:,edges(i,2));
    l = line([xj(1),yj(1)],[xj(2),yj(2)],[xj(3),yj(3)],...
        'Color','blue','LineWidth',1.0,'LineStyle','--');
    h = [h,l];
end

end