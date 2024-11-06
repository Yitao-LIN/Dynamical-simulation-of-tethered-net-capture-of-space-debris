clc
% clear
% load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\test\netpos.txt');
% load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\test\netlabel.txt');
% load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\test\tarpos.txt');
row = 16;
column = 16;
node_num = (row+1) * (column + 1) + 4;
edge_num = (row+1) * column + (column + 1) * row + 4;

timepoint = size(netpos, 1) / node_num;
figure(1)

hold on
set (gca,'position', [0.1,0.1,0.88,0.88] );
set(gcf, 'unit', 'normalized', 'position', [0.1, 0.1, 0.7, 0.80]);
 m = Mesh('ball_origin');
for i = 400 : 100 : timepoint
    clf
    scatter3(netpos(1 + (i -1 ) * node_num: i * node_num, 1), netpos(1 + (i -1 ) * node_num: i * node_num, 2), netpos(1 + (i -1 ) * node_num: i * node_num, 3),  2, 'filled', 'MarkerEdgeColor',[0.27451 0.5098 0.70588], 'MarkerFaceColor',[0.27451 0.5098 0.70588]);
    hold on
    axis([-3 3 -3 3 -2 4])
    
    for k = 1 : edge_num
        netpos_couple = [netpos((i - 1) * node_num + netlabel(k, 1), 1), netpos((i - 1) * node_num + netlabel(k, 2), 1);
            netpos((i - 1) * node_num + netlabel(k, 1), 2), netpos((i - 1) * node_num + netlabel(k, 2), 2);
            netpos((i - 1) * node_num + netlabel(k, 1), 3), netpos((i - 1) * node_num + netlabel(k, 2), 3)];
        len = dist(netpos_couple(:, 1), netpos_couple(:, 2));
        if len<= 0.20
            plot3(netpos_couple(1, :), netpos_couple(2, :), netpos_couple(3, :), 'k', 'LineWidth', 0.6);
        else
            plot3(netpos_couple(1, :), netpos_couple(2, :), netpos_couple(3, :), 'color', [1, 0.01 / len, 0], 'LineWidth', 0.6);
        end
        
        hold on
    end

       trimesh(m.F, m.V(:,1) + tarpos(i, 1), m.V(:,2)+ tarpos(i, 2), m.V(:,3)+ tarpos(i, 3),'LineWidth',0.5,'EdgeColor','k');
%      m.draw
%      plot_cuboid(tarpos(i, 1 : 3) + [0.5, -0.5, -1] , tarpos(i, 1 : 3) + [-0.5, 0.5, 1]);
%           plot_cuboid([0 0 1.02] + [0.5, -0.5, -1] ,[0 0 1.02]+ [-0.5, 0.5, 1]);
           text(-2, 2, 3, [num2str(i * 0.01), 's'], 'FontSize', 20)
        view([1 0 0])
%       [x, y, z]=sphere(25);
%      surf(0.5 * x + tarpos(i, 1) * ones(26), 0.5 * y + tarpos(i, 2)* ones(26), 0.5 * z + + tarpos(i, 3)* ones(26), 'FaceAlpha',0.2, 'EdgeColor', 'none');
     pause(0.0001)
%     filename = 'T:\#Rifle在学习\# Graduation Thesis\进度\coding\0. Deployment Simulation\output\goodoutput\17x17_cube_4.gif';
%     frame = getframe(1);
%     im = frame2im(frame);
%     [A, map] = rgb2ind(im, 256);
%     if i == 1
%         imwrite(A, map, filename, 'gif', 'DelayTime', 0.001);
%     else
%         imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.001);
%     end
    
end

% plot(1 : 9 : 9000, netpos(1 : 9 : 9000, 3));

function dis = dist(a, b)
dis = sqrt( (a(1)-b(1))^2 + (a(2)-b(2))^2 + (a(3)-b(3))^2 );
end

function plot_cuboid(start_point,final_point)
%% 根据起点和终点，计算长方体的8个的顶点
vertexIndex=[0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
cuboidSize=final_point-start_point;             %方向向量
vertex=repmat(start_point,8,1)+vertexIndex.*repmat(cuboidSize,8,1);
%% 定义6个平面分别对应的顶点
facet=[1 2 4 3;1 2 6 5;1 3 7 5;2 4 8 6;3 4 8 7;5 6 8 7];
%% 定义8个顶点的颜色，绘制的平面颜色根据顶点的颜色进行插补
color=[1;1;1;1;1;1;1;1];
%% 绘制并展示图像
patch('Vertices',vertex,'Faces',facet,'FaceVertexCData',color,'FaceColor','interp','FaceAlpha',0.5);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
%% 设置xyz显示范围
% xmin=min(vertex(:,1))-1;
% xmax=max(vertex(:,1))+1;
% ymin=min(vertex(:,2))-1;
% ymax=max(vertex(:,2))+1;
% zmin=min(vertex(:,3))-1;
% zmax=max(vertex(:,3))+1;
% axis([xmin xmax ymin ymax zmin zmax]) 
end