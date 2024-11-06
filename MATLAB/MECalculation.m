clc
clear
% verification of conservation of mechanical energy
load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\netpos.txt');
load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\netlabel.txt');
load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\tarpos.txt');
load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\netvel.txt');
load('T:\#Rifle在学习\# Graduation Thesis\进度\coding\2Tether_net_to_TriMesh\output\mass.txt');

% load('acceleration.txt')

row = 16;
column =16;
node_num = (row+1) * (column + 1) + 4;
edge_num = (row+1) * column + (column + 1) * row + 4;
stiffness = 12566.0;

timepoint = size(netpos, 1) / node_num;
figure(1)
set (gca,'position', [0.1,0.1,0.88,0.88] );
set(gcf, 'unit', 'normalized', 'position', [0.1, 0.1, 0.7, 0.7]);

Kinetic = zeros(1, timepoint );
Potential = zeros(1, timepoint );
Mechanical = zeros(1, timepoint );
 Gravity_potential = zeros(1, timepoint );
  MaxPos = zeros(1, timepoint );
for i = 1 : timepoint 
    for j = 1 : node_num
        Kinetic(i) = Kinetic(i) + 1/2 * mass(j) * ( netvel((i - 1)*node_num + j, 1)^2 + netvel((i - 1)*node_num + j, 2)^2 + netvel((i - 1)*node_num + j, 3)^2);
%         Gravity_potential(i) =  Gravity_potential(i) + netpos((i - 1)*node_num + j, 3) * 9.8 * mass(j);
    end
    for k = 1 : edge_num
        netpos_couple = [netpos((i - 1) * node_num + netlabel(k, 1), 1), netpos((i - 1) * node_num + netlabel(k, 2), 1);
                                        netpos((i - 1) * node_num + netlabel(k, 1), 2), netpos((i - 1) * node_num + netlabel(k, 2), 2);
                                        netpos((i - 1) * node_num + netlabel(k, 1), 3), netpos((i - 1) * node_num + netlabel(k, 2), 3)];
        len = dist(netpos_couple(:, 1), netpos_couple(:, 2));
        
        if len > 0.25
            Potential(i) = Potential(i) + 1/2 * stiffness * (len-0.25)^2;
        end
    end
    Mechanical(i) = Potential(i) + Kinetic(i); %+ Gravity_potential(i);
end
for i = 1 : timepoint
    MaxPos(i) = max(abs(netvel((i-1) * node_num + 1 : i * node_num, 3)));
end
plot(1 : timepoint , Mechanical);
figure(2)
plot(1 : timepoint, MaxPos);
% figure(2)
% plot(1: timepoint, netvel(1 : 29 : 39875, 3));
% 
% for i = 1 : 68
%     y = netvel(23156 * 68 + i : 68 : 2040000, 1);
%     x = 1 : length(y);
%     plot(x, y);
%     hold on
% end

function dis = dist(a, b)
    dis = sqrt( (a(1)-b(1))^2 + (a(2)-b(2))^2 + (a(3)-b(3))^2 );
end