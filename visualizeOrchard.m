function visualizeOrchard(env, figureNum)
%% 可视化果园环境
% 输入: env - 果园环境结构体
%       figureNum - 图窗编号 (可选，默认当前图窗)

if nargin < 2
    figureNum = gcf;
end

figure(figureNum);
hold on;

%% 绘制地面/果园网格
xRange = linspace(env.bounds.xMin, env.bounds.xMax, 20);
yRange = linspace(env.bounds.yMin, env.bounds.yMax, 20);
[X, Y] = meshgrid(xRange, yRange);
Z = zeros(size(X));  % 地面高度

% 绘制草地
surf(X, Y, Z, 'FaceColor', [0.4, 0.7, 0.4], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

%% 绘制果树行列线
% 果树行线
for row = env.bounds.xMin:400:env.bounds.xMax
    plot3([row, row], [env.bounds.yMin, env.bounds.yMax], [0, 0], '--', ...
        'Color', [0.6, 0.6, 0.6], 'LineWidth', 0.5);
end

%% 绘制障碍物
cylRes = 20;  % 圆柱分辨率

for i = 1:size(env.obstacles, 1)
    pos = env.obstacles(i, :);
    obsType = env.obsType(i);
    radius = env.obsRadius(i);
    height = env.obsHeight(i);
    
    switch obsType
        case 1  % 树干 - 圆柱
            color = env.vis.trunkColor;
            % 绘制圆柱
            [x, y, z] = cylinder(radius, cylRes);
            z = z * height + (pos(3) - height/2);
            x = x + pos(1);
            y = y + pos(2);
            surf(x, y, z, 'FaceColor', color, 'FaceAlpha', 0.9, 'EdgeColor', 'none');
            
        case 2  % 树冠 - 椭球/球
            color = env.vis.treeColor;
            % 绘制椭球（简化为球体缩放）
            [x, y, z] = sphere(cylRes);
            % 缩放为椭球
            x = x * radius + pos(1);
            y = y * radius + pos(2);
            z = z * (height/2) + pos(3);
            surf(x, y, z, 'FaceColor', color, 'FaceAlpha', 0.6, 'EdgeColor', 'none');
            
        case 3  % 移动机器人 - 圆柱
            color = env.vis.robotColor;
            [x, y, z] = cylinder(radius, cylRes);
            z = z * height + (pos(3) - height/2);
            x = x + pos(1);
            y = y + pos(2);
            surf(x, y, z, 'FaceColor', color, 'FaceAlpha', 0.8, 'EdgeColor', 'k');
            
            % 绘制移动方向箭头
            if i <= size(env.obsVelocity, 1) && norm(env.obsVelocity(i, :)) > 0
                vel = env.obsVelocity(i, :) * 5;  % 缩放显示
                quiver3(pos(1), pos(2), pos(3)+height/2, ...
                    vel(1), vel(2), vel(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            end
            
        case 4  % 果实 - 小球
            color = env.vis.fruitColor;
            [x, y, z] = sphere(10);
            x = x * radius + pos(1);
            y = y * radius + pos(2);
            z = z * radius + pos(3);
            surf(x, y, z, 'FaceColor', color, 'FaceAlpha', 0.9, 'EdgeColor', 'none');
    end
end

%% 绘制起点和终点
plot3(env.source(1), env.source(2), env.source(3), 'bs', 'MarkerSize', 15, ...
    'MarkerFaceColor', 'b', 'DisplayName', '起点(入口)');
plot3(env.goal(1), env.goal(2), env.goal(3), 'r^', 'MarkerSize', 15, ...
    'MarkerFaceColor', 'r', 'DisplayName', '终点(采摘点)');

%% 绘制边界框
plot3([env.bounds.xMin, env.bounds.xMax, env.bounds.xMax, env.bounds.xMin, env.bounds.xMin], ...
      [env.bounds.yMin, env.bounds.yMin, env.bounds.yMax, env.bounds.yMax, env.bounds.yMin], ...
      [env.bounds.zMin, env.bounds.zMin, env.bounds.zMin, env.bounds.zMin, env.bounds.zMin], ...
      'k--', 'LineWidth', 1);

%% 设置视角和标签
title(sprintf('果园环境 (%s场景) - %d个障碍物', env.scenarioType, size(env.obstacles, 1)));
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
axis equal;
grid on;
view(-45, 25);

%% 添加图例
legend('Location', 'best');

%% 光照效果
camlight('headlight');
lighting gouraud;
material dull;

hold off;

end
