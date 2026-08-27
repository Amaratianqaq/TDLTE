%% diag_ber.m - 关键诊断: Turbo译码后的误码率 (离成功有多近?)
% 用已保存的 rx_accum_result.mat 跑完整收端链, 输出:
%   - 译码后BER (译码输出 vs 发送bits)
%   - CRC
% BER<1%  -> 很近, 加迭代/调LLR可能翻盘
% BER~50% -> 译码器未锁住, 信号不足
% 另外输出均衡后符号的"中位数SNR"(vs 均值), 看是否有大部分子载波其实不错。

clear; clc;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

if ~exist('rx_accum_result.mat','file')
    error('没有 rx_accum_result.mat, 请先运行 rx_accum');
end
S = load('rx_accum_result.mat');
rxAvg = S.rxAvg(:).';
prm = GetLTEParams();

%% 1) 完整收端链
[bits, crcOK] = OFDM_RxMain(rxAvg, prm);

%% 2) 发送参考bits
rng(42);
[txBits, ~] = TBGen(prm);
txBits = txBits(:);
b = min(numel(bits), numel(txBits));
ber = mean(bits(1:b) ~= txBits(1:b));
fprintf('Turbo译码后: BER = %.4f, CRC = %d\n', ber, crcOK);
if ber < 0.01 && crcOK == 0
    fprintf('>> BER很低但CRC=0! 极可能是最后几个bit错(CRC24A) -> 加迭代/微调即可翻盘!\n');
elseif ber < 0.05
    fprintf('>> BER=%.1f%% -> 比较近, 加大迭代/调LLR缩放可能成功。\n', ber*100);
elseif ber < 0.2
    fprintf('>> BER=%.1f%% -> 译码器部分锁住, 信号边缘。\n', ber*100);
else
    fprintf('>> BER=%.1f%% -> 译码器未锁住, 信号严重不足。\n', ber*100);
end

%% 3) 均衡后每子载波SNR分布 (均值 vs 中位数)
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
[rxAl, ~, ~, ~] = syncLTE(rxAvg, txIQ, prm);
rxGrid = OFDMDemod(rxAl, prm);
[rs0, rs1] = RefSignalGen(prm);
H = ChanEst(rxGrid, rs0, rs1, prm);
notchMask = false(prm.NSubc,1); notchMask(451:750) = true;
for l = 1:prm.NSym
    h = H(:,l); h(notchMask) = NaN;
    h = movmean(h, 64, 'omitnan');
    h(notchMask) = H(notchMask,l);
    H(:,l) = h;
end
pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;
clean = ~notchMask;
residual = [abs(rxGrid(clean,pilotIdx1)-H(clean,pilotIdx1).*rs0(clean)).^2; ...
            abs(rxGrid(clean,pilotIdx2)-H(clean,pilotIdx2).*rs1(clean)).^2];
noiseVar = max(median(residual), 1e-9);
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
dataSyms = ExtractData(eqGrid, prm);

% 发送参考符号
[bits2, info] = TBGen(prm);
bitsCRC = CRCAttach(bits2, '24A');
[cbs, segInfo] = CodeBlockSeg(bitsCRC);
cbOutputs = cell(segInfo.C, 1);
for r = 1:segInfo.C
    coded = TurboEncode(cbs{r}, segInfo.K(r));
    cbOutputs{r} = RateMatch(coded, segInfo.K(r), segInfo.C, info.G, info.Qm, prm.NL, prm.RV);
end
codedBits = CodeBlockCat(cbOutputs);
scrambled = Scramble(codedBits, prm);
symRef = Modulate(scrambled, prm.Modulation);
txGrid2 = ResMap(symRef, rs0, rs1, prm);
txDataSyms = ExtractData(txGrid2, prm);

err = dataSyms - txDataSyms;
snrPer = 10*log10(abs(txDataSyms).^2 ./ (abs(err).^2 + eps));
fprintf('\n均衡后每符号SNR: 均值=%.2f dB | 中位数=%.2f dB | 90分位=%.2f dB\n', ...
    mean(snrPer), median(snrPer), quantile(snrPer, 0.9));
fprintf('SNR>0dB的符号占比 = %.1f%%\n', 100*mean(snrPer > 0));
logResult('diag_ber', struct('ber',ber,'crc',crcOK,'snrMed',median(snrPer),'snr90',quantile(snrPer,0.9)));
