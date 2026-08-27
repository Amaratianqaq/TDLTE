%% snr_floor_test.m - 定位离线链路SNR门槛过高的原因 (纯离线, 无需硬件)
% 三个环节对比 (均为AWGN):
%   A 全链路   : txIQ+噪声 -> 完整收端 (当前代码, 含LS信道估计/噪声估计)
%   B 理想信道 : 同A但 H=1 + 精确噪声方差 (隔离信道估计/噪声方差估计的损耗)
%   C 纯Turbo  : 编码->BPSK+AWGN->软解调->Turbo译码 (隔离Turbo/速率匹配/解扰损耗)
% 输出: 各环节 CRC=1 的最低SNR。门槛明显偏高的环节就是损耗所在。
% 参考: QPSK+码率~0.55 Turbo 理论上子载波/编码bit SNR ~0~4dB 即可过。

clear; clc;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
Ptx = mean(abs(txIQ).^2);
N = numel(txIQ);

%% A vs B: OFDM全链路 (A=当前+频域平滑信道估计, B=理想信道)
fprintf('==== A/B: OFDM链路 (SNR为总SNR, 子载波SNR=总SNR+2.3dB) ====\n');
fprintf('SNR(dB) | A平滑估计 | B理想信道\n');
SNRs = 0:1:10;
for s = SNRs
    rng(1);
    Pn = Ptx / 10^(s/10);
    nz = sqrt(Pn/2) * (randn(1,N) + 1j*randn(1,N));
    rxIQ = txIQ + nz;

    prmA = GetLTEParams();
    prmA.chanSmooth = 128;                % 频域平滑信道估计(128子载波窗)
    [~, crcA] = OFDM_RxMain(rxIQ, prmA);

    prmB = GetLTEParams();
    prmB.idealChan = true;
    prmB.noiseVarOverride = 2048 * Pn;    % FFT后每bin噪声方差=2048*Pn
    [~, crcB] = OFDM_RxMain(rxIQ, prmB);

    fprintf('%2d      | %d          | %d\n', s, crcA, crcB);
end

%% C: 纯Turbo链路 (编码->BPSK+AWGN->软解调->Turbo)
fprintf('\n==== C: 纯Turbo链路 (BPSK+AWGN, 无OFDM) ====\n');
rng(42);
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[cbs, segInfo] = CodeBlockSeg(bitsCRC);
cbOut = cell(segInfo.C, 1);
for r = 1:segInfo.C
    coded = TurboEncode(cbs{r}, segInfo.K(r));
    cbOut{r} = RateMatch(coded, segInfo.K(r), segInfo.C, info.G, info.Qm, prm.NL, prm.RV, r);
end
rm = CodeBlockCat(cbOut);
scr = Scramble(rm, prm);
% BPSK: 按Turbo译码LLR约定(负LLR=bit0, 正LLR=bit1): bit0->-1, bit1->+1
sym = 2 * double(scr(:)) - 1;
G = numel(sym);
fprintf('编码bit数=%d, 有效码率=%.3f\n', G, numel(bits)/G);

Krs = [repmat(segInfo.Km, segInfo.Cm, 1); repmat(segInfo.Kp, segInfo.Cp, 1)];
fprintf('编码bitSNR(dB) | CRC | BER\n');
for sb = 0:0.5:8
    N0 = 10^(-sb/10);
    sigma2 = N0 / 2;
    y = sym + sqrt(sigma2) * randn(size(sym));
    llr = 2 * y / sigma2;          % BPSK软解调
    desc = DescrambleSoft(llr, prm);
    ptr = 1; cbDec = cell(1, segInfo.C);
    for r = 1:segInfo.C
        Kr = Krs(r);
        E = rateMatchEr(G, prm.Qm, prm.NL, segInfo.C, r);
        cbBits = desc(ptr:ptr+E-1); ptr = ptr + E;
        rmOut = DeRateMatch(cbBits, Kr, prm.RV);
        [cbDec{r}, ~] = TurboDecode(rmOut, Kr, prm.maxIter);
    end
    [bitsDec, crcC] = CodeBlockDeSeg(cbDec, segInfo);
    ber = mean(bits(:) ~= bitsDec(:));
    fprintf('%4.1f       | %d   | %.4g\n', sb, crcC, ber);
end

%% 记录结果到日志
logResult('snr_floor_test', struct('note','阈值比较: A全链路 vs B理想信道 vs C纯Turbo'));
