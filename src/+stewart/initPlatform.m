function params = initPlatform(params)
%INITPLATFORM 由平台几何参数生成并缓存基座点 A 与动平台点 B
%  - A_O: 基座铰点 A_i，在 {O} 坐标系表达，尺寸 3x6
%  - B_P: 动平台铰点 B_i，在 {P} 坐标系表达，尺寸 3x6
%
% 约定：
%   下坐标系 {O}：基座平面，z=0
%   上坐标系 {P}：动平台平面，z=0（平台局部坐标）

rA = params.geom.rA;
rB = params.geom.rB;

thetaA = params.geom.thetaA;
thetaB = params.geom.thetaB;

assert(numel(thetaA) == 6 && numel(thetaB) == 6, ...
    "thetaA/thetaB 必须都是 1x6（对应 6 个铰点）");

% A_O：基座点在 {O}，z=0
A_O = zeros(3,6);
for i = 1:6
    A_O(:,i) = [rA*cos(thetaA(i)); rA*sin(thetaA(i)); 0];
end

% B_P：平台点在 {P}，z=0
B_P = zeros(3,6);
for i = 1:6
    B_P(:,i) = [rB*cos(thetaB(i)); rB*sin(thetaB(i)); 0];
end

% 缓存
params.cache.A_O = A_O;
params.cache.B_P = B_P;
params.cache.connB = params.geom.connB;
end