clc
clear
load('velosity.txt');
load('position.txt');

% theo
m = 1;
g = 9.8;
l_0 = 1.0;
k = 500;
c = 1;
starter = [m*g/k, 0];
omega_n = sqrt(k/m);
c_critical = 2 * sqrt(k * m);
damping_ratio = c / c_critical;
t = 0;
t_end = 2.0;
t_step = 0.01;
i = 1;
A = (starter(2) + (damping_ratio + sqrt(damping_ratio^2 - 1)) * omega_n * starter(1)) / (2 * omega_n * sqrt(damping_ratio^2 - 1));
B = (-starter(2) - (damping_ratio - sqrt(damping_ratio^2 - 1)) * omega_n * starter(1)) / (2 * omega_n * sqrt(damping_ratio^2 - 1));
omega_d = omega_n * sqrt(1 - damping_ratio^2);
while(t < t_end)
    x_theo(i) = exp(-damping_ratio * omega_n * t) * ...
              ((((starter(2) + damping_ratio * omega_n * starter(1))/(omega_d)) * sin(omega_d * t))+...
              (starter(1) * cos(omega_d * t)));
    i = i + 1;
    t = t + t_step;
end
x_theo = 1-m*g/k + x_theo;
% end theo


x = ones(length(position), 1);
z = ones(length(position), 1);

spring_bend = 9;
spring_x = repmat([0.975, 1.025], 1, spring_bend);
spring_x = [spring_x, 1];
spring_z = ones(1, spring_bend * 2 + 1);

[sphere_x, sphere_y, sphere_z] = sphere(100);
hold on

for i = 1 : length(position)
    surf(sphere_x * 0.015 + x(i), sphere_y * 0.01 + position(i) + 0.01, sphere_z * 0.015 + z(i), 'LineStyle','none','FaceColor', [0.6902, 0.76863, 0.87059])
    material metal
    hold on
    surf(sphere_x * 0.015 + x(i) + 0.2, sphere_y * 0.01 + x_theo(i) + 0.01, sphere_z * 0.015 + z(i), 'LineStyle','none','FaceColor', [0.6902, 0.76863, 0.87059])
    material metal
    camlight('headlight')
%     plot3(x(i), position(i), z(i), 'or', 'MarkerSize', 12);
    hold on 
    %绘制弹簧
    spring_y = 0.85 : (position(i)-0.85) / (spring_bend*2) : position(i);
    plot3(spring_x, spring_y, spring_z, 'LineWidth', 5);
    hold on
    spring_y_theo = 0.85 : (position(i)-0.85) / (spring_bend*2) : position(i);
    plot3(spring_x + 0.2, spring_y, spring_z, 'LineWidth', 5);
    hold on
    
%     plot3([1 1], [0 position(i)], [1 1], 'r');
    axis([0.9, 1.3, 0.9, 1.1 0.9 1.1])
    xlabel('x', 'FontSize', 18)
    ylabel('y', 'FontSize', 18)
    zlabel('z')
%     legend('数值解', '理论解')
    view(2)
    pause(0.01)
    
%     filename = 'single_spring_damped_compare.gif';
%     frame = getframe(1);
%     im = frame2im(frame);
%     [A, map] = rgb2ind(im, 256);
%         if i == 1
%             imwrite(A, map, filename, 'gif', 'DelayTime', 0.001);
%         else
%             imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.001);
%         end
        
    clf
end

figure(2)
plot(0 : 0.01 : 0.01 * (length(position) - 1), position, 'b<', 'LineWidth', 0.8)
hold on
plot(T, Y(:, 1), 'r-')
title('Damping vibration simulation', 'FontSize', 24)
xlabel('t/s', 'FontSize', 24)
ylabel('x', 'FontSize', 24)
legend('数值解', '理论解', 'FontSize', 24)