function [J, c_num] = jacobian(pose, params)
%JACOBIAN 6x6 Stewart 主雅可比：Ldot = J * [v; omega]
% pose = [x y z roll pitch yaw] 用于计算 R_OP 与几何量
% v, omega 均默认为在 {O} 表达

pose = pose(:).';
assert(numel(pose)==6, "pose 必须是 [x y z roll pitch yaw]");

p_O = pose(1:3).';
roll = pose(4); pitch = pose(5); yaw = pose(6);

A_O = params.cache.A_O;   % 3x6
B_P = params.cache.B_P;   % 3x6

R_OP = stewart.rotmZYX(roll, pitch, yaw);

J = zeros(6,6);

for i = 1:6
    % r_PB 在 {O} 的表达（从 P 原点指向第 i 个平台铰点 B_i）
    r_PB_O = R_OP * B_P(:,i);

    % 腿向量（从 A_i 指向 B_i），注意要加平移 p_O
    l_O = p_O + r_PB_O - A_O(:,i);

    Li = norm(l_O);
    if Li < 1e-9
        error("第 %d 条腿长度过小，检查 pose/几何参数", i);
    end
    s_i = l_O / Li;  % 单位方向

    % 行装配：前三列 s_i'，后三列 (r_PB_O × s_i)'  —— 与文章一致
    J(i,1:3) = s_i.';
    J(i,4:6) = cross(r_PB_O, s_i).';
end

c_num = cond(J);

% 预警
if isfield(params,"constraints") && isfield(params.constraints,"condWarn")
    if c_num > params.constraints.condWarn
        warning("Jacobian condition number %.3e > condWarn %.3e (接近奇异)", ...
            c_num, params.constraints.condWarn);
    end
end
end