
%% verify_hw_link.m - 真实RF环回链路诊断脚本
% 用途: 当"无警告但CRC=false"时, 区分问题出在哪一环:
%   1) 信号是否真的回来了?           -> 互相关归一化系数 rho
%   2) 采集窗口与子帧边界是否对齐?   -> CP相关定时, 给出帧偏移 off*
%   3) 是否存在载波频偏?             -> CP相关频偏估计 (Hz)
%   4) 对齐/频偏校正后CRC能否通过?   -> 分别尝试 原始/仅对齐/对齐+去频偏
% 运行前请先确认 XSRP上位机中: TX_Freq == RX_Freq (任务书默认2.38GHz),
%   环回模式正确、天线连接、衰减/增益合理。
% 运行: 在MATLAB中 cd 到本目录后直接运行本脚本 (无需硬件之外的依赖)

clear; clc; close all;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

%% 1) 参数与发送信号 (与TD_LTE_Main一致)
prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm);

%% 2) 真实RF环回
pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';
rxIQ = OFDM_RFLoopback(txIQ, pcip, xsrpip);      % 真实环回 (30720样点)

save('hw_capture.mat', 'txIQ', 'rxIQ', 'prm');  % 保存, 便于离线复分析

fprintf('==== 硬件环回诊断 ====\n');
fprintf('txIQ功率=%.4g, rxIQ功率=%.4g, 直流分量|mean(rxIQ)|=%.4g\n', ...
    mean(abs(txIQ).^2), mean(abs(rxIQ).^2), abs(mean(rxIQ)));

%% 3~7) 稳健同步: 频偏搜索 + 互相关定时 (syncLTE, 不依赖被污染CP估计)
[rxAligned, rho, cfoHz, delay] = syncLTE(rxIQ, txIQ, prm);
fprintf('稳健同步: 频偏=%.1f Hz, 帧偏移delay=%d样点, 对齐后互相关rho=%.4f\n', ...
    cfoHz, delay, rho);

% 备选: 若解包的I/Q次序与设备回传不符(rxIQ≈conj(txIQ)), 直接相关会很低
c2 = xcorr(conj(rxIQ(:).'), txIQ(:).');
rhoC = max(abs(c2)) / sqrt(sum(abs(rxIQ(:).').^2) * sum(abs(txIQ(:).').^2));
fprintf('共轭备选rhoC=%.4f\n', rhoC);

if rho < 0.15 && rhoC >= rho
    fprintf('\n>> 提示: rho低但rhoC高 -> 接收解包I/Q次序疑似反了(OFDM_RxDataDeal对设备回传字节序的假设与设备不符)。\n');
end
if max(rho, rhoC) < 0.15
    fprintf(['\n>> 警告: 相关过低, 收到的信号与发送信号基本不相关(噪声/异频/格式错)。\n', ...
        '   优先检查(按概率): \n', ...
        '   1) XSRP上位机 TX频率 与 RX频率 是否相等 (SystemConfig.ini当前TX=953.6MHz RX=908.6MHz, 差45MHz! 任务书默认均为2.38GHz);\n', ...
        '   2) 运行方式是否为真实系统/环回模式是否正确;\n', ...
        '   3) 天线是否连接, 发射衰减是否过大(当前40dB), 接收增益是否过小(当前21dB);\n', ...
        '   4) Get_IQ采集的时隙号(当前01)是否与tx_command发射时隙对齐;\n', ...
        '   5) OFDM_RxDataDeal解包的每样点字节序/位数与设备回传格式是否一致。\n']);
end

%% 8) 应用对齐(循环移位)与去频偏, 分别跑收端看CRC
r = rxIQ(:).' - mean(rxIQ);
rxOnlyTim = r(mod((0:numel(r)-1) + delay, numel(r)) + 1);   % 仅定时对齐(不去频偏)

fprintf('\n==== 收端CRC对比 ====\n');
crc0 = NaN; crc1 = NaN; crc2 = NaN;
try
    [~, crc0] = OFDM_RxMain(rxIQ, prm);
    fprintf('原始接收信号      : CRC=%d\n', crc0);
catch ME
    fprintf('原始接收信号      : 出错 %s\n', ME.message);
end
try
    [~, crc1] = OFDM_RxMain(rxAligned, prm);
    fprintf('去频偏+定时对齐    : CRC=%d\n', crc1);
catch ME
    fprintf('去频偏+定时对齐    : 出错 %s\n', ME.message);
end
try
    [~, crc2] = OFDM_RxMain(rxOnlyTim, prm);
    fprintf('仅定时对齐         : CRC=%d\n', crc2);
catch ME
    fprintf('仅定时对齐         : 出错 %s\n', ME.message);
end

%% 9) 星座图对比 (均衡后)
figure('Name', '硬件环回诊断');
subplot(1,2,1); plot(real(txIQ), imag(txIQ), '.'); axis equal; grid on;
title('发送时域IQ'); xlabel('I'); ylabel('Q');
subplot(1,2,2); plot(real(rxAligned), imag(rxAligned), '.'); axis equal; grid on;
title('接收时域IQ(对齐+去频偏)'); xlabel('I'); ylabel('Q');

%% 10) 记录结果到日志
logResult('verify_hw_link', struct('rho',rho,'rhoC',rhoC,'cfoHz',cfoHz,'delay',delay, ...
    'rxpow',mean(abs(rxIQ(:)).^2),'crc_raw',crc0,'crc_align',crc1,'crc_timing',crc2));
