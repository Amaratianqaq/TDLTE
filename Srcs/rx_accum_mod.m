function rx_accum_mod()
% rx_accum_mod - K帧相干平均 (可选调制 + 发送峰值)
% 用途: 64QAM 在 txPeak=200 时 BER≈0.0001(约8个bit错), CRC需0个错。
%       若这8个错含随机噪声成分, K帧平均可把它们压到0 -> CRC=1。
%       若平均后 BER 不变(确定性残余), 说明是设备低频内容的固有地板。
% 使用: 直接运行, 输入调制/txPeak/K。直连+1GHz+当前增益状态。

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

modStr = input('调制方式(QPSK/16QAM/64QAM, 默认64QAM): ', 's');
if isempty(modStr), modStr = '64QAM'; end
tpStr = input('txPeak(默认200): ', 's');
tp = str2double(tpStr); if isnan(tp), tp = 200; end
Kstr = input('平均帧数K(默认16): ', 's');
K = str2double(Kstr); if isnan(K) || K < 1, K = 16; end

prm = GetLTEParams();
switch upper(modStr)
    case 'QPSK',  prm.Modulation = 'QPSK';  prm.Qm = 2; prm.TBsize = 15840;
    case '16QAM', prm.Modulation = '16QAM'; prm.Qm = 4; prm.TBsize = 28336;
    case '64QAM', prm.Modulation = '64QAM'; prm.Qm = 6; prm.TBsize = 55056;
    otherwise, error('不支持的调制 %s', modStr);
end
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';

fprintf('==== K=%d 相干平均 (调制=%s, txPeak=%d) ====\n', K, modStr, tp);
sumAcc = zeros(1, numel(txIQ));
rhos = zeros(1, K);
for k = 1:K
    rxIQ = OFDM_RFLoopback(txIQ, '192.168.1.180', '192.168.1.166', tp, 0);
    rxIQ = rxIQ(:).';
    [rs, rho, ~, ~, ph] = syncLTE(rxIQ, txIQ, prm);
    sumAcc = sumAcc + rs .* exp(-1j*ph);
    rhos(k) = rho;
    if mod(k, 4) == 0 || k == 1
        fprintf('第%3d帧 rho=%.4f (均值=%.4f)\n', k, rho, mean(rhos(1:k)));
    end
end

Davg = sumAcc / K;
[~, rhoAvg] = syncLTE(Davg, txIQ, prm);
snrPer = subcarrierSNR(Davg, txIQ, prm);
[bits, crcOK] = OFDM_RxMain(Davg, prm);
rng(42); [txBits, ~] = TBGen(prm);
ber = mean(bits(1:numel(txBits)) ~= txBits(1:numel(txBits)));

fprintf('\n==== 结果 ====\n');
fprintf('单帧rho均值=%.4f | 平均后rho=%.4f\n', mean(rhos), rhoAvg);
fprintf('每子载波SNR: 平均=%.2f dB | 中位=%.2f dB | 最差=%.2f dB (子载波%d)\n', ...
    mean(snrPer), median(snrPer), min(snrPer), find(snrPer == min(snrPer), 1));
fprintf('BER=%.5f (错%d bit) | CRC=%d\n', ber, sum(bits(1:numel(txBits)) ~= txBits(1:numel(txBits))), crcOK);
if crcOK == 1
    fprintf('>>>>> %s K=%d 打通! <<<<<\n', modStr, K);
else
    fprintf('>> 若BER比单帧(0.0001)明显下降但未到0 -> 加大K; 若不变 -> 确定性残余, 需另想办法。\n');
end

save('rx_accum_mod_result.mat', 'Davg', 'snrPer', 'ber', 'crcOK', 'rhoAvg', 'rhos');
logResult('rx_accum_mod', struct('mod',modStr,'tp',tp,'K',K,'rhoAvg',rhoAvg,'ber',ber,'crc',crcOK));
fprintf('结果已存 rx_accum_mod_result.mat\n');
end
