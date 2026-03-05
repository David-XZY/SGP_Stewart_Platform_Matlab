params = platform_default();
params = stewart.initPlatform(params);

A_O = params.cache.A_O;
B_P = params.cache.B_P;

disp(size(A_O));   % 应为 3 6
disp(size(B_P));   % 应为 3 6

disp(A_O);
disp(B_P);