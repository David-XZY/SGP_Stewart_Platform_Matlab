% startup.m  
disp("== Project startup: StewartUCU ==");

% 1) 基本环境
clearvars; close all; clc;

% 2) 加载默认参数
if exist("platform_default", "file")
    params = platform_default();
    params = stewart.initPlatform(params);
    assignin("base", "params", params);
    assignin("base", "pose0", params.init.pose);
    assignin("base", "vel0",  params.init.vel);
    assignin("base", "acc0",  params.init.acc);
end

% 3) 准备输出目录
if ~exist(fullfile(pwd, "data", "logs"), "dir")
    mkdir(fullfile(pwd, "data", "logs"));
end