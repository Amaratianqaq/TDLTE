%% rx_sweep_txpeak.m - 扫发送峰值找"无削波"工作点 (解包修复后)
% 背景: 修复I路解包后 rho 0.3->0.8, 但 RX 满幅削波(PAPR~1.14)导致 CRC 仍为0。
%       削波 = 发送峰值2047 + 接收增益过高 -> ADC饱和。
% 本脚本: 扫 txPeak(发送峰值), 对每个值测 RX峰值/PAPR/满幅占比/rho/每子载波SNR/CRC,
%       找 PAPR 回升到~3、CRC=1 的最小 txPeak。
% 前置: 直连+1GHz+当前增益状态, 勿碰设备/线缆。
% 使用: 直接运行, 每个峰值约3~5秒。

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

pcip = '192.168.1.180'; xsrpip = '192.168.1.166';
prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';

txPeaks = [2047 1000 500 300 200 150 100 75 50];
fprintf('==== 扫发送峰值 (找无削波工作点) ====\n');
fprintf('%-6s | %-8s | %-8s | %-8s | %-8s | %-9s | %-6s | %-5s\n', ...
    'txPeak', 'RX峰值', 'PAPR', '满幅%', 'rho', 'SNR(dB)', 'BER', 'CRC');

for tp = txPeaks
    rxIQ = OFDM_RFLoopback(txIQ, pcip, xsrpip, tp, 0);
    rxIQ = rxIQ(:).';
    r = abs(rxIQ);
    rxPeak = max(r); rmsv = sqrt(mean(r.^2)); papr = rxPeak / max(rmsv, 1e-12);
    clipFrac = 100 * mean(r > 0.9);
    [rs, rho, ~, ~] = syncLTE(rxIQ, txIQ, prm);
    snrPer = subcarrierSNR(rs, txIQ, prm);
    try
        [bits, crcOK] = OFDM_RxMain(rs, prm);
        rng(42); [txBits, ~] = TBGen(prm);
        b = min(numel(bits), numel(txBits));
        ber = mean(bits(1:b) ~= txBits(1:b));
    catch ME
        crcOK = NaN; ber = NaN; fprintf('CRC错: %s\n', ME.message);
    end
    fprintf('%-6d | %-8.3f | %-8.2f | %-8.1f | %-8.4f | %-9.2f | %-6.4f | %-5d\n', ...
        tp, rxPeak, papr, clipFrac, rho, mean(snrPer), ber, crcOK);
    if crcOK == 1
        fprintf('>>>>> txPeak=%d 时 CRC=1! 链路打通 <<<<<\n', tp);
        save('rx_sweep_txpeak_best.mat', 'rxIQ', 'rs', 'tp', 'snrPer', 'ber');
    end
end

fprintf('\n>> 判读: PAPR回升到~3且满幅占比<30%%的行 = 无削波工作点, 该行 CRC 应为1。\n');
fprintf('   若 RX峰值 仍恒为1.414(所有行) -> 降 RX增益(上位机RX1_Gain)后再扫, 而非继续降txPeak。\n');
logResult('rx_sweep_txpeak', struct('note','扫发送峰值找无削波工作点'));
