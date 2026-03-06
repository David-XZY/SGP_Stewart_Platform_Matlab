params = platform_default();
params = stewart.initPlatform(params);

dt = 0.01;
Tsim = 5.0;
N = round(Tsim/dt);

% 初始
pose = params.init.pose;
L_act = stewart.ik(pose, params).';  % 6x1 作为"编码器读数"
L_cmd = L_act;

% 参考：0.5s 时给一个小阶跃
pose_ref0 = pose;
pose_ref1 = pose + [0.02 0 0  0 0 deg2rad(3)];

% 外环增益（单位 1/s），先从小到大调
Kp_pos = 2.0;      % 位置
Kp_ang = 2.0;      % 姿态（rad）

% 内环一阶时间常数（电缸速度环+位置环等效）
Tact = 0.05;

% 记录
log.t = (0:N-1)*dt;
log.pose = zeros(N,6);
log.ref  = zeros(N,6);
log.kappa = zeros(N,1);

optsFK = struct("tol",1e-10,"maxIter",10,"verbose",false);

for k = 1:N
    t = (k-1)*dt;
    pose_ref = pose_ref0;
    if t > 0.5
        pose_ref = pose_ref1;
    end

    % 位姿误差（角度做 wrap，避免跳变）
    e = pose_ref - pose;
    e(4:6) = atan2(sin(e(4:6)), cos(e(4:6)));

    % 外环：扭量命令 u=[v;w]
    u = [Kp_pos*e(1:3), Kp_ang*e(4:6)].';  % 6x1

    % Jacobian → 腿长速度指令
    [J, kappa] = stewart.jacobian(pose, params);
    Ldot_cmd = J * u;

    % 积分得到 L_cmd（加个简单限幅，防止数值爆）
    L_cmd = L_cmd + Ldot_cmd*dt;

    % 内环电缸一阶跟踪：L_act 向 L_cmd 收敛
    L_act = L_act + (dt/Tact) * (L_cmd - L_act);

    % 由"编码器 L_act"反解当前 pose（用上一步 pose 作初值）
    [pose, ~] = stewart.fk_nr(L_act, pose, params, optsFK);

    % 记录
    log.pose(k,:) = pose;
    log.ref(k,:)  = pose_ref;
    log.kappa(k)  = kappa;
end

% 画误差
err = log.ref - log.pose;
err(:,4:6) = atan2(sin(err(:,4:6)), cos(err(:,4:6)));

figure("Color","w"); grid on; hold on;
plot(log.t, err(:,1)); plot(log.t, err(:,2)); plot(log.t, err(:,3));
legend("ex","ey","ez"); xlabel("t (s)"); ylabel("pos error (m)"); title("Position error");

figure("Color","w"); grid on; hold on;
plot(log.t, rad2deg(err(:,4))); plot(log.t, rad2deg(err(:,5))); plot(log.t, rad2deg(err(:,6)));
legend("eroll","epitch","eyaw"); xlabel("t (s)"); ylabel("angle error (deg)"); title("Angle error");

figure("Color","w"); grid on;
plot(log.t, log.kappa); xlabel("t (s)"); ylabel("cond(J)"); title("Jacobian condition number");