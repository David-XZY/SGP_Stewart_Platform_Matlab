params = platform_default();
params = stewart.initPlatform(params);

pose = [0 0 0.8  0 0 0];   % 先给一个简单位姿：上平台在z=0.8，无姿态
[L, s_O] = stewart.ik(pose, params);

disp(L);
disp(s_O);