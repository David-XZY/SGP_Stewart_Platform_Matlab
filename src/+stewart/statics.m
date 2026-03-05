function tau = statics(pose, F_ext, params, mode)
%STATICS  静力学载荷映射：F_ext = J^T * tau
%
% 输入:
%   pose  : [x y z roll pitch yaw]
%   F_ext : [Fx Fy Fz Mx My Mz] (1x6 或 6x1)，在 {O} 表达
%   params: 你的参数结构体（含 A_O、B_P cache）
%   mode  : (可选) "generate" 或 "balance"
%           - "generate"(默认): tau =  J^{-T} * F_ext
%           - "balance"       : tau = -J^{-T} * F_ext   % 抵消外载荷
%
% 输出:
%   tau   : 1x6，各缸需要提供的轴向推/拉力（单位 N）

if nargin < 4
    mode = "generate";
end

F_ext = F_ext(:);               % 强制 6x1
assert(numel(F_ext)==6, "F_ext 必须是 6 维 [Fx Fy Fz Mx My Mz]");

[J, ~] = stewart.jacobian(pose, params);
JT = J';

% 数值保护：接近奇异时用伪逆降级（与文档一致）
if rcond(JT) < 1e-10
    warning("Statics: J' is near singular (rcond=%.3e), using pinv.", rcond(JT));
    tau_col = pinv(JT) * F_ext;
else
    tau_col = JT \ F_ext;       % 左除比 inv 更稳
end

if mode == "balance"
    tau_col = -tau_col;
end

tau = tau_col.';                % 变回 1x6，方便数据流
end