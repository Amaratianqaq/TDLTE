%% rx_accum.m - 多帧相干平均 (纯代码提升SNR 10*log10(K) dB)
% 原理: 发送同一数据K次, 采集K帧; 每帧用syncLTE同步(去频偏+定时+相位对齐)后
%       相干平均。噪声/干扰帧间独立 -> 平均后SNR提升 10*log10(K) dB:
%         K=4 -> +6dB, K=8 -> +9dB, K=16 -> +12dB
% 用途: 在不调天线/不改硬件的情况下, 直接补上OFDM子载波信噪比缺口。
% 前置: 上位机设置保持最优(衰减2dB/增益5dB左右), 天线姿态固定。
% 注意: 每次约5~10秒(含同步频偏搜索), K=8约1分钟, K=16约2~3分钟。

clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

Kstr = input('平均帧数K(默认8, 推荐8~16): ', 's');
K = str2double(Kstr); if isnan(K) || K < 1, K = 8; end

prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm);
txIQ = txIQ(:).';

fprintf('==== 多帧相干平均 K=%d (请勿碰天线/设备) ====\n', K);
frames = zeros(1, numel(txIQ), K);
rhos = zeros(1, K);
for k = 1:K
    rxIQ = OFDM_RFLoopback(txIQ, '192.168.1.180', '192.168.1.166', 2047, 0);
    rxIQ = rxIQ(:).';
    [rxAl, rho, ~, ~, phase] = syncLTE(rxIQ, txIQ, prm);
    frames(1, :, k) = rxAl .* exp(-1j*phase);   % 相位对齐后存入
    rhos(k) = rho;
    fprintf('第%2d帧: rho=%.4f\n', k, rho);
end

%% 相干平均
rxAvg = mean(frames, 3);
[~, rhoAvg] = syncLTE(rxAvg, txIQ, prm);
fprintf('\n单帧rho均值=%.4f | 平均后rho=%.4f (理论提升约 %.1f dB)\n', ...
    mean(rhos), rhoAvg, 10*log10(K));

%% 直接跑收端看CRC
try
    [bits, crcOK] = OFDM_RxMain(rxAvg, prm);
    fprintf('平均后CRC=%d\n', crcOK);
catch ME
    fprintf('CRC计算出错: %s\n', ME.message);
    crcOK = [];
end
save('rx_accum_result.mat', 'rxAvg', 'rhos', 'rhoAvg', 'crcOK');
fprintf('结果已存 rx_accum_result.mat\n');

if crcOK
    fprintf('\n>> CRC=1! 多帧平均成功, 链路打通。\n');
else
    fprintf('\n>> 若平均后rho明显提升但仍CRC=0, 可加大K(16/32)再试;\n');
    fprintf('   或结合天线姿态优化(把单帧rho先推高, 平均效果更好)。\n');
end

%% 记录结果到日志
logResult('rx_accum', struct('K',K,'rhoMean',mean(rhos),'rhoAvg',rhoAvg,'crc',crcOK));
