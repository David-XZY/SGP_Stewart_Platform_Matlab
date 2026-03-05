function params = platform_default()
% 平台默认参数入口：几何 + 约束 + 动力学参数

%% -------- 几何参数（单位：m / rad）--------
params.geom.rA = 0.60;   % 固定基座外接圆半径 r_A
params.geom.rB = 0.4;   % 动平台外接圆半径 r_B

% 基座：三对分别围绕 0°,120°,240°，每对间隔 2*alphaA
% 平台：三对围绕 60°,180°,300°，每对间隔 2*alphaB
params.geom.alphaA = deg2rad(15);
params.geom.alphaB = deg2rad(15);

cA = deg2rad([0, 120, 240]);
cB = deg2rad([60, 180, 300]);

params.geom.thetaA = reshape([cA - params.geom.alphaA; cA + params.geom.alphaA], 1, []);
params.geom.thetaB = reshape([cB - params.geom.alphaB; cB + params.geom.alphaB], 1, []);
params.geom.thetaB = params.geom.thetaB([6 1 2 3 4 5]);

%% -------- 初始状位姿--------
% pose = [x y z roll pitch yaw] (m, rad)
params.init.pose = [0 0 0.8  0 0 0];

% vel  = [vx vy vz wx wy wz] (m/s, rad/s)
params.init.vel  = zeros(1,6);

% acc  = [ax ay az dwx dwy dwz] (m/s^2, rad/s^2)
params.init.acc  = zeros(1,6);

%% -------- 约束/阈值--------
params.constraints.condWarn = 1e4;  % cond(J)>1e4 

%% -------- 动力学参数--------
params.dyn.g  = [0;0;-9.81];
params.dyn.Mp = 25.0;
params.dyn.Ip = diag([1.2 1.2 2.5]);


%% cache
params.cache = struct();
end