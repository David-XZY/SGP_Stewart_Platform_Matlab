% scripts/run_00_check_init.m  （升级版：初始化 + 可视化）

params = platform_default();
params = stewart.initPlatform(params);

% 你想画的一个"名义位姿"（可以随时改）
pose = params.init.pose;

% ===== 清晰打印 A1-6（{O}）与 B1-6（{P}）以及 B1-6（{O}）=====
A_O = params.cache.A_O;   % 3x6, 基座点在{O}
B_P = params.cache.B_P;   % 3x6, 平台点在{P}

% 位姿（与你绘图用同一个 pose）
p_O  = pose(1:3).';
R_OP = stewart.rotmZYX(pose(4), pose(5), pose(6));
B_O  = p_O + R_OP * B_P;  % 平台点映射到{O}，方便直观查看

fprintf("\n================== Point Coordinates ==================\n");
fprintf("Convention: A_i in {O}, B_i in {P}, and B_i mapped to {O} using pose\n");
fprintf("pose = [x y z roll pitch yaw] = [% .4f % .4f % .4f  % .4f % .4f % .4f]\n", pose);

% 打印表头
fprintf("\n%-3s | %12s %12s %12s || %12s %12s %12s || %12s %12s %12s\n", ...
    "ID", "A_Ox", "A_Oy", "A_Oz", "B_Px", "B_Py", "B_Pz", "B_Ox", "B_Oy", "B_Oz");
fprintf("%s\n", repmat('-', 1, 3+1+12*3+4+12*3+4+12*3));

% 逐点打印（对齐、易读）
for i = 1:6
    fprintf("%-3d | %12.6f %12.6f %12.6f || %12.6f %12.6f %12.6f || %12.6f %12.6f %12.6f\n", ...
        i, ...
        A_O(1,i), A_O(2,i), A_O(3,i), ...
        B_P(1,i), B_P(2,i), B_P(3,i), ...
        B_O(1,i), B_O(2,i), B_O(3,i));
end

% 额外：打印每个点的极角（便于核对编号顺序）
thetaA_deg = rad2deg(atan2(A_O(2,:), A_O(1,:)));
thetaB_deg = rad2deg(atan2(B_P(2,:), B_P(1,:)));
fprintf("\nAngles (deg):\n");
fprintf("thetaA (A points around base)   = %s\n", mat2str(thetaA_deg, 4));
fprintf("thetaB (B points around platform)= %s\n", mat2str(thetaB_deg, 4));
fprintf("========================================================\n\n");

% 生成示意图并保存
outPng = fullfile("data","logs","run00_schematic.png");
opts = struct();
opts.showLabels = true;
opts.showPlanes = true;
opts.axisLength = 0.20;
opts.savePath = outPng;
opts.title = "Stewart schematic (O/P, A/B, legs)";

stewart.plotSchematic(pose, params, opts);

disp("Saved: " + outPng);