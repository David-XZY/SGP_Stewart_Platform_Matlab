% scripts/run_02_demo_jacobian.m
params = platform_default();
params = stewart.initPlatform(params);

pose = params.init.pose;

[J, kappa] = stewart.jacobian(pose, params);
disp("cond(J) = " + kappa);

% -------- A) 平移列数值验证：omega=0 --------
dt = 1e-6;
v  = [0.01; -0.02; 0.03];
omega = [0;0;0];

L0 = stewart.ik(pose, params);
pose2 = pose; pose2(1:3) = pose(1:3) + (v.'*dt);
L2 = stewart.ik(pose2, params);

Ldot_num  = ((L2 - L0).'/dt);
Ldot_pred = J * [v; omega];

disp("Check translation columns (pred vs num):");
disp([Ldot_pred, Ldot_num]);

% -------- B) 零姿态下旋转列验证：仅 yaw（此时欧拉率≈omega）--------
omega = [0;0;0.1]; % rad/s about z in {O}
pose3 = pose; pose3(6) = pose(6) + omega(3)*dt; % yaw += wz*dt
L3 = stewart.ik(pose3, params);

Ldot_num_r  = ((L3 - L0).'/dt);
Ldot_pred_r = J * [0;0;0; omega];

disp("Check yaw rotation column at zero angles (pred vs num):");
disp([Ldot_pred_r, Ldot_num_r]);