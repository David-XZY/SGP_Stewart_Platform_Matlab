% scripts/run_03_demo_statics.m
params = platform_default();
params = stewart.initPlatform(params);

pose = [0 0 0.8  0 0 0];
[J, kappa] = stewart.jacobian(pose, params);
disp("cond(J) = " + kappa);

% 例子：平台原点受到一个向下的力 100N（无力矩）
F_ext = [0 0 -100  0 0 0];

% 1) "产生该外力"的缸力（对应文档 tau = J^{-T}F_ext）
tau = stewart.statics(pose, F_ext, params, "generate");
disp("tau (generate) = "); disp(tau);

% 2) 检验：J' * tau ≈ F_ext
F_check = (J' * tau.').';
disp("F_check = "); disp(F_check);

disp("error norm = " + norm(F_check - F_ext));