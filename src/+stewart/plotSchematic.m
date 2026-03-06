function fig = plotSchematic(pose, params, opts)
%PLOTSCHEMATIC  Stewart 平台示意图（上下平面/坐标系/支链/铰点）
%
% pose = [x y z roll pitch yaw]  (m, rad)
% params.cache.A_O (3x6), params.cache.B_P (3x6)
% params.geom.connB (1x6) 可选：第 i 条腿连接到第 connB(i) 个 B 点
%
% opts fields (可选):
%   .showLabels (default true)
%   .showPlanes (default true)
%   .axisLength (default 0.2)  坐标轴显示长度（m）
%   .savePath   (default "")   若非空则保存图片（png）
%   .title      (default "")

if nargin < 3, opts = struct(); end
if ~isfield(opts,"showLabels"), opts.showLabels = true; end
if ~isfield(opts,"showPlanes"), opts.showPlanes = true; end
if ~isfield(opts,"axisLength"), opts.axisLength = 0.2; end
if ~isfield(opts,"savePath"),   opts.savePath   = ""; end
if ~isfield(opts,"title"),      opts.title      = ""; end

pose = pose(:).';
assert(numel(pose)==6, "pose 必须是 [x y z roll pitch yaw]");

A_O = params.cache.A_O;
B_P = params.cache.B_P;

% 连线映射（缺省就是 A_i 连 B_i）
if isfield(params,"geom") && isfield(params.geom,"connB")
    connB = params.geom.connB;
elseif isfield(params,"cache") && isfield(params.cache,"connB")
    connB = params.cache.connB;
else
    connB = 1:6;
end
connB = connB(:).';
assert(numel(connB)==6, "connB 必须是 1x6");

% 位姿
p_O = pose(1:3).';
R_OP = stewart.rotmZYX(pose(4), pose(5), pose(6));

% 计算平台点在 {O} 的坐标：B_O_all(:,j) = p_O + R_OP*B_P(:,j)
B_O_all = p_O + R_OP * B_P;

% 图窗
fig = figure("Name","Stewart Schematic","Color","w");
ax = axes(fig); %#ok<LAXES>
hold(ax,"on"); grid(ax,"on"); axis(ax,"equal");
view(ax, 3);
xlabel(ax,"X_O (m)"); ylabel(ax,"Y_O (m)"); zlabel(ax,"Z_O (m)");

% -------- 画基座平面 / 平台平面（可选）--------
if opts.showPlanes
    % 基座平面：用一个圆盘（根据 A 点半径估计）
    rA = max(vecnorm(A_O(1:2,:),2,1));
    t = linspace(0,2*pi,120);
    baseCircle = [rA*cos(t); rA*sin(t); zeros(size(t))];
    patch(ax, baseCircle(1,:), baseCircle(2,:), baseCircle(3,:), ...
        0.95*ones(1,3), "FaceAlpha",0.15, "EdgeColor","none");

    % 平台平面：用一个圆盘（根据 B 点半径估计）并按 R_OP 旋转+平移
    rB = max(vecnorm(B_P(1:2,:),2,1));
    topCircle_P = [rB*cos(t); rB*sin(t); zeros(size(t))];
    topCircle_O = p_O + R_OP * topCircle_P;
    patch(ax, topCircle_O(1,:), topCircle_O(2,:), topCircle_O(3,:), ...
        0.75*ones(1,3), "FaceAlpha",0.15, "EdgeColor","none");
end

% -------- 画铰点 --------
plot3(ax, A_O(1,:), A_O(2,:), A_O(3,:), "o", "LineWidth",1.5);
plot3(ax, B_O_all(1,:), B_O_all(2,:), B_O_all(3,:), "s", "LineWidth",1.5);

% -------- 画支链（腿）--------
for i = 1:6
    j = connB(i);
    P1 = A_O(:,i);
    P2 = B_O_all(:,j);
    plot3(ax, [P1(1) P2(1)], [P1(2) P2(2)], [P1(3) P2(3)], "-", "LineWidth",1.6);
end

% -------- 画坐标系 O 与 P --------
L = opts.axisLength;

% O 坐标系
O = [0;0;0];
quiver3(ax, O(1),O(2),O(3), L,0,0, "LineWidth",2, "MaxHeadSize",0.6);
quiver3(ax, O(1),O(2),O(3), 0,L,0, "LineWidth",2, "MaxHeadSize",0.6);
quiver3(ax, O(1),O(2),O(3), 0,0,L, "LineWidth",2, "MaxHeadSize",0.6);
text(ax, O(1),O(2),O(3), "  O", "FontWeight","bold");

% P 坐标系（P 原点在 {O} 下为 p_O，轴方向为 R_OP 的列向量）
Px = p_O + R_OP*[L;0;0];
Py = p_O + R_OP*[0;L;0];
Pz = p_O + R_OP*[0;0;L];
quiver3(ax, p_O(1),p_O(2),p_O(3), Px(1)-p_O(1),Px(2)-p_O(2),Px(3)-p_O(3), "LineWidth",2, "MaxHeadSize",0.6);
quiver3(ax, p_O(1),p_O(2),p_O(3), Py(1)-p_O(1),Py(2)-p_O(2),Py(3)-p_O(3), "LineWidth",2, "MaxHeadSize",0.6);
quiver3(ax, p_O(1),p_O(2),p_O(3), Pz(1)-p_O(1),Pz(2)-p_O(2),Pz(3)-p_O(3), "LineWidth",2, "MaxHeadSize",0.6);
text(ax, p_O(1),p_O(2),p_O(3), "  P", "FontWeight","bold");

% -------- 标签（可选）--------
if opts.showLabels
    for i = 1:6
        text(ax, A_O(1,i), A_O(2,i), A_O(3,i), sprintf("  A%d",i));
    end
    for j = 1:6
        text(ax, B_O_all(1,j), B_O_all(2,j), B_O_all(3,j), sprintf("  B%d",j));
    end
    for i = 1:6
        j = connB(i);
        mid = 0.5*(A_O(:,i) + B_O_all(:,j));
        text(ax, mid(1), mid(2), mid(3), sprintf("  L%d",i));
    end
end

% 标题
if opts.title ~= ""
    title(ax, opts.title, "Interpreter","none");
end

% 视域自动适配
allPts = [A_O, B_O_all, O, p_O];
mins = min(allPts,[],2); maxs = max(allPts,[],2);
pad = 0.15*max(1e-6, max(maxs-mins));
xlim(ax, [mins(1)-pad, maxs(1)+pad]);
ylim(ax, [mins(2)-pad, maxs(2)+pad]);
zlim(ax, [mins(3)-pad, maxs(3)+pad]);

% 保存
if opts.savePath ~= ""
    exportgraphics(fig, opts.savePath, "Resolution", 300);
end
end