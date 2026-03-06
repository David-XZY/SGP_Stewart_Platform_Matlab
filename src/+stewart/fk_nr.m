function [pose, info] = fk_nr(L_target, pose0, params, opts)
%FK_NR  Stewart 正运动学（NR迭代）：求 pose 使得 IK(pose)=L_target
%
% 输入:
%   L_target : 1x6 或 6x1 目标腿长
%   pose0    : 初值 [x y z roll pitch yaw]
%   params   : 参数（含 A_O, B_P）
%   opts     : 可选结构体
%       .tol      (default 1e-10)
%       .maxIter  (default 30)
%       .rcondMin (default 1e-12)
%       .verbose  (default false)
%
% 输出:
%   pose : 求得位姿
%   info : 迭代信息

if nargin < 4, opts = struct(); end
if ~isfield(opts,"tol"),      opts.tol = 1e-10; end
if ~isfield(opts,"maxIter"),  opts.maxIter = 30; end
if ~isfield(opts,"rcondMin"), opts.rcondMin = 1e-12; end
if ~isfield(opts,"verbose"),  opts.verbose = false; end

L_target = L_target(:);
assert(numel(L_target)==6, "L_target 必须是 6 维");

pose = pose0(:).';
assert(numel(pose)==6, "pose0 必须是 6 维");

info = struct();
info.converged = false;
info.iter = 0;
info.errNorm = NaN;
info.condJ = NaN;

for k = 1:opts.maxIter
    L_pred = stewart.ik(pose, params);   % 1x6
    err = L_pred(:) - L_target;          % 6x1

    eNorm = norm(err);
    info.iter = k;
    info.errNorm = eNorm;

    if opts.verbose
        fprintf("FK iter %d: ||err||=%.3e\n", k, eNorm);
    end

    if eNorm < opts.tol
        info.converged = true;
        break;
    end

    [J, cnum] = stewart.jacobian(pose, params);
    info.condJ = cnum;

    % 求解 dX： err ≈ J * dX  =>  dX = -J^{-1} err
    if rcond(J) < opts.rcondMin
        dX = -pinv(J) * err;
    else
        dX = -(J \ err);
    end

    % 更新 pose
    pose(1:3) = pose(1:3) + dX(1:3).';
    pose(4:6) = wrapToPiLocal(pose(4:6) + dX(4:6).');
end

end

function a = wrapToPiLocal(a)
% 不依赖工具箱的 wrapToPi
a = atan2(sin(a), cos(a));
end