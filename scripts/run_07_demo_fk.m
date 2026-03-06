params = platform_default();
params = stewart.initPlatform(params);

pose_true = params.init.pose + [0.02 -0.01 0.00  0 0 deg2rad(5)];
L = stewart.ik(pose_true, params);

pose_guess = params.init.pose;   % 初值用名义位姿
opts = struct("tol",1e-10,"maxIter",30,"verbose",false);

[pose_est, info] = stewart.fk_nr(L, pose_guess, params, opts);

disp("pose_true = "); disp(pose_true);
disp("pose_est  = "); disp(pose_est);
disp("pose_err  = "); disp(pose_est - pose_true);
disp(info);