function [L, s_O, l_O, B_O] = ik(pose, params)
%IK  Stewart 逆运动学（腿长）
% 输入:
%   pose = [x y z roll pitch yaw]  (位置在{O}，姿态用ZYX欧拉角)
% 输出:
%   L   : 1x6 腿长
%   s_O : 3x6 腿方向单位向量（在{O}）
%   l_O : 3x6 腿向量（在{O}），从A指向B
%   B_O : 3x6 平台铰点在{O}的坐标

pose = pose(:).';   % 强制成 1x6
assert(numel(pose)==6, "pose 必须是 [x y z roll pitch yaw]");

x = pose(1); y = pose(2); z = pose(3);
roll = pose(4); pitch = pose(5); yaw = pose(6);

A_O = params.cache.A_O;   % 3x6
B_P = params.cache.B_P;   % 3x6

p_O = [x; y; z];          % r_OP（P原点在O中）
R_OP = stewart.rotmZYX(roll, pitch, yaw);

L   = zeros(1,6);
s_O = zeros(3,6);
l_O = zeros(3,6);
B_O = zeros(3,6);

for i = 1:6
    j = params.cache.connB(i);
    B_O(:,i) = p_O + R_OP * B_P(:,j);   % 平台点从{P}转到{O}再平移
    l_O(:,i) = B_O(:,i) - A_O(:,i);     % 腿向量
    Li = norm(l_O(:,i));
    if Li < 1e-9
        error("第 %d 条腿长度过小，检查 pose 或几何参数", i);
    end
    L(i) = Li;
    s_O(:,i) = l_O(:,i) / Li;
end
end