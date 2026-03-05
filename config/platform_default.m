function params = platform_default()
% 平台默认参数入口：几何 + 约束 + 动力学参数

%% -------- 几何参数（单位：m / rad）--------
params.geom.rA = 0.60;   % 固定基座外接圆半径 r_A
params.geom.rB = 0.35;   % 动平台外接圆半径 r_B

% 基座：三对分别围绕 0°,120°,240°，每对间隔 2*alphaA
% 平台：三对围绕 60°,180°,300°，每对间隔 2*alphaB
params.geom.alphaA = deg2rad(20);
params.geom.alphaB = deg2rad(20);

cA = deg2rad([0, 120, 240]);
cB = deg2rad([60, 180, 300]);

params.geom.thetaA = reshape([cA - params.geom.alphaA; cA + params.geom.alphaA], 1, []);
params.geom.thetaB = reshape([cB - params.geom.alphaB; cB + params.geom.alphaB], 1, []);

params.geom.connB = [2 1 4 3 6 5];

%% -------- 约束/阈值--------
params.constraints.condWarn = 1e4;  % 文中示例阈值：cond(J)>1e4 

%% -------- 动力学参数--------
params.dyn.g  = [0;0;-9.81];
params.dyn.Mp = 25.0;
params.dyn.Ip = diag([1.2 1.2 2.5]);


%% cache
params.cache = struct();
end