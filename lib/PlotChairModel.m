clc
clear 
close all

model = ComputeChairClass('Conf',0.5);
mu = model.mu;
C = model.C;
edges = model.edges;

num_kpts = size(mu,2);

figure;

for i = 1:num_kpts
    h = error_ellipse(C(:,:,i),mu(:,i),'conf',model.conf);
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
    
    line(x(1,:),x(2,:),x(3,:),'Color',[1,0,0,0.5],'LineWidth',2);
    hold on
end


axis equal 
axis off
