params = platform_default();
params = stewart.initPlatform(params);

pose = params.init.pose;   
[L, s_O] = stewart.ik(pose, params);

disp(L);
disp(s_O);