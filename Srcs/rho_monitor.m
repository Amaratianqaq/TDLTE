%% rho_monitor.m - 连续监测OFDM回传相关度
% 每次运行: 环回 + 同步(syncLTE) + 计算 rho, 实时显示并记录。
% 用途:
%   - 观察天线/设备状态随时间是否稳定
%   - 调上位机参数后快速对比前后 rho
%   - 找出 rho 最高的时刻/状态
% 注意: 运行期间勿碰天线/设备; 保持上位机设置不变(当前最佳: 衰减15/增益5)。
% 使用: 直接运行, 输入次数(默认5), 每次约5~10秒。

clear; clc; close all;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

Nstr = input('运行次数(默认5): ', 's');
N = str2double(Nstr); if isnan(N) || N < 1, N = 5; end

prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm);
txIQ = txIQ(:).';

% 发送峰值与PAPR (与最近试验一致: 2047/不削减; 可自行修改)
txPeak = 2047;
papr   = 0;

%% 实时图
f = figure('Name','OFDM回传相关度监测','Position',[100 100 900 480],'NumberTitle','off');
ax = axes(f); hold(ax,'on'); grid(ax,'on');
xlabel(ax,'运行次数'); ylabel(ax,'rho');
title(ax,'每次环回的互相关度 rho (同步后)');
ylim(ax,[0 1]);

log = [];
fprintf('==== rho 监测 x%d (峰值=%d, PAPR削减=%g, 勿碰天线/设备) ====\n', N, txPeak, papr);
for k = 1:N
    % 1) 真实环回
    rxIQ = OFDM_RFLoopback(txIQ, '192.168.1.180', '192.168.1.166', txPeak, papr);
    rxIQ = rxIQ(:).';

    % 2) 稳健同步 + 互相关
    [~, rho, cfoHz, delay] = syncLTE(rxIQ, txIQ, prm);

    % 3) 附加指标
    Nf = numel(rxIQ);
    X  = fft(txIQ, Nf);  Y = fft(rxIQ, Nf);
    fq = (0:Nf-1)/Nf*30.72e6;  fq(fq>15.36e6) = fq(fq>15.36e6) - 30.72e6;
    band = abs(fq) < 9.5e6 & abs(fq) > 0.2e6;
    smX = movmean(abs(X),150);  smY = movmean(abs(Y),150);
    cc = corrcoef(smX(band), smY(band));  rhoEnv = cc(1,2);
    rxPow = mean(abs(rxIQ).^2);
    clipFrac = mean(abs(rxIQ) > 0.9*max(abs(rxIQ)));

    log = [log; struct('run',k,'rho',rho,'rhoEnv',rhoEnv,'rxPow',rxPow, ...
        'clip',clipFrac,'cfo',cfoHz,'delay',delay)]; %#ok<AGROW>

    % 4) 显示
    fprintf('第%2d次: rho=%.4f | 包络=%.3f | rx功率=%.3f | 削波=%.1f%% | 频偏=%.0fHz | 帧偏移=%d\n', ...
        k, rho, rhoEnv, rxPow, clipFrac*100, cfoHz, delay);
    bar(ax, 1:k, [log.rho], 'b');
    drawnow;
end

%% 汇总
fprintf('\n==== 汇总 ====\n');
fprintf('rho   : min=%.4f  max=%.4f  mean=%.4f\n', ...
    min([log.rho]), max([log.rho]), mean([log.rho]));
fprintf('包络  : min=%.3f  max=%.3f\n', min([log.rhoEnv]), max([log.rhoEnv]));
[bestRho, ib] = max([log.rho]);
fprintf('最佳: 第%d次 rho=%.4f (频偏=%.0fHz, 帧偏移=%d)\n', ...
    log(ib).run, bestRho, log(ib).cfo, log(ib).delay);
save('rho_monitor_log.mat', 'log');

%% 记录结果到日志
logResult('rho_monitor', struct('rhoMin',min([log.rho]),'rhoMax',max([log.rho]), ...
    'rhoMean',mean([log.rho]),'bestRun',log(ib).run,'bestRho',bestRho));
fprintf('结果已存 rho_monitor_log.mat\n');
