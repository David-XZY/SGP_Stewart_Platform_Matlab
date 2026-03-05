function tau_dyn = invDynamics_vw(pose, vel, acc, params, F_ext)
%INVDYNAMICS_VW  逆动力学（虚功法）：tau = -J^{-T} * (F_pf + F_legs_projected + F_ext)
%
% 输入:
%   pose : [x y z roll pitch yaw]  (rad)
%   vel  : [vx vy vz wx wy wz]     (m/s, rad/s) —— 建议在 {O} 表达
%   acc  : [ax ay az dwx dwy dwz]  (m/s^2, rad/s^2) —— 建议在 {O} 表达
%   params: 含 params.dyn.Mp, params.dyn.Ip, params.dyn.g + cache
%   F_ext: (可选) 外部广义力/力矩 [Fx Fy Fz Mx My Mz] (N, Nm)，在 {O} 表达
%
% 输出:
%   tau_dyn: 1x6 各缸轴向推/拉力 (N)

if nargin < 5 || isempty(F_ext)
    F_ext = zeros(6,1);
end
F_ext = F_ext(:);
assert(numel(F_ext)==6, "F_ext 必须是 6 维");

% -------- 1) 取动力学参数 --------
Mp = params.dyn.Mp;
Ip = params.dyn.Ip;      % 这里按文档用常量惯量张量 :contentReference[oaicite:5]{index=5}
g  = params.dyn.g;

% -------- 2) 拆状态 --------
pose = pose(:).'; vel = vel(:).'; acc = acc(:).';
assert(numel(pose)==6 && numel(vel)==6 && numel(acc)==6, "pose/vel/acc 都必须是 6 维");

p_O   = pose(1:3).';
roll  = pose(4); pitch = pose(5); yaw = pose(6);

v_O   = vel(1:3).';
w_O   = vel(4:6).';
a_O   = acc(1:3).';
dw_O  = acc(4:6).';

% -------- 3) Jacobian（你现有实现已含 connB 映射） --------
[J, ~] = stewart.jacobian(pose, params);
JT = J';

% -------- 4) 计算 F_pf（动平台主体等效惯性扳手）--------
% 4a 平动部分：F_trans = Mp*g - Mp*a_p   :contentReference[oaicite:6]{index=6}
F_trans = Mp * g - Mp * a_O;

% 4b 转动部分：文档给的是  M_rot = -Ip*dw - w×(Ip*w) :contentReference[oaicite:7]{index=7}
% 为了让输出扳手和 J 同在 {O}，这里采用：先把角速度/角加速度近似转到 {P} 再算，再转回 {O}
R_OP = stewart.rotmZYX(roll, pitch, yaw);

w_P  = R_OP.' * w_O;         % 近似（忽略 Rdot 项）
dw_P = R_OP.' * dw_O;

M_rot_P = - Ip * dw_P - cross(w_P, Ip * w_P);   % 文档形式
M_rot_O = R_OP * M_rot_P;

F_pf = [F_trans; M_rot_O];

% -------- 5) 支腿附加项（文档默认置零） --------
% 文档说明：工程上常用"零质量连杆/折算质量"简化，把该项置零 :contentReference[oaicite:8]{index=8}
F_legs_projected = zeros(6,1);

% -------- 6) 虚功重映射： (J^T)*tau + F_pf + F_legs + F_ext = 0 --------
rhs = F_pf + F_legs_projected + F_ext;

if rcond(JT) < 1e-10
    warning("invDynamics_vw: J' near singular (rcond=%.3e), using pinv.", rcond(JT));
    tau_col = - pinv(JT) * rhs;            % :contentReference[oaicite:9]{index=9}
else
    tau_col = - (JT \ rhs);                % :contentReference[oaicite:10]{index=10}
end

tau_dyn = tau_col.';  % 1x6
end