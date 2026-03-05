% scripts/run_05_demo_invDynamics.m
params = platform_default();
params = stewart.initPlatform(params);

pose = [0 0 0.8  0 0 0];
vel  = zeros(1,6);
acc  = zeros(1,6);

tau_dyn = stewart.invDynamics_vw(pose, vel, acc, params);   % 默认仅重力
disp("tau_dyn = "); disp(tau_dyn);

% 对照：静力学"产生向上支撑力 Mg"
F_support = [0 0 (-params.dyn.Mp*params.dyn.g(3))  0 0 0];  % = [0 0 Mp*9.81 0 0 0]
tau_sta = stewart.statics(pose, F_support, params, "generate");
disp("tau_sta (support Mg) = "); disp(tau_sta);

disp("diff norm = " + norm(tau_dyn - tau_sta));