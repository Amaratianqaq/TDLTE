%% evm_phase.m - 测试"残余相位/增益校正"能否提升EVM (无需硬件)
% 现象: rho=0.32 (相干强) 但 EVM=-1.3dB (符号对不齐?) -> 疑似均衡后
%       符号存在整体/逐符号的幅度相位偏移。用LS最小二乘校正后再测EVM。
%   v5: 整体增益/相位校正 (rx = rx/alpha, alpha=LS最优)
%   v6: 逐符号增益/相位校正 (每符号独立alpha)
% 若v5/v6明显优于v1(-1.3dB) -> 残余相位是主因, 把校正加入OFDM_RxMain。

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

%% 发送参考
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[cbs, segInfo] = CodeBlockSeg(bitsCRC);
cbOutputs = cell(segInfo.C, 1);
for r = 1:segInfo.C
    coded = TurboEncode(cbs{r}, segInfo.K(r));
    cbOutputs{r} = RateMatch(coded, segInfo.K(r), segInfo.C, info.G, info.Qm, prm.NL, prm.RV);
end
codedBits = CodeBlockCat(cbOutputs);
scrambled = Scramble(codedBits, prm);
sym = Modulate(scrambled, prm.Modulation);
[rs0, rs1] = RefSignalGen(prm);
txGrid = ResMap(sym, rs0, rs1, prm);
txDataSyms = ExtractData(txGrid, prm);

%% 接收处理 (与当前链一致: 平滑64+污染隔离+MMSE)
[rxAl, rho, ~, ~] = syncLTE(rxAvg, txIQ, prm);
rxGrid = OFDMDemod(rxAl, prm);
H = ChanEst(rxGrid, rs0, rs1, prm);
notchMask = false(prm.NSubc,1);
kNotch = 451:750; kNotch = kNotch(kNotch>=1 & kNotch<=prm.NSubc);
notchMask(kNotch) = true;
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

%% v1 基线
snr1 = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(dataSyms - txDataSyms).^2));

%% v5 整体LS增益/相位校正
alpha = sum(conj(txDataSyms) .* dataSyms) / sum(abs(txDataSyms).^2);
ds5 = dataSyms / alpha;
snr5 = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(ds5 - txDataSyms).^2));

%% v6 逐符号校正
Nsc = prm.NSubc; NsymData = prm.NSym - prm.NPilot;
txM = reshape(txDataSyms, Nsc, NsymData);
rxM = reshape(dataSyms, Nsc, NsymData);
ds6 = zeros(size(rxM));
for s = 1:NsymData
    a = sum(conj(txM(:,s)) .* rxM(:,s)) / sum(abs(txM(:,s)).^2);
    ds6(:,s) = rxM(:,s) / a;
end
ds6 = ds6(:);
snr6 = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(ds6 - txDataSyms).^2));

%% v7 逐符号+整体 (先逐符号再整体)
alpha7 = sum(conj(txDataSyms) .* ds6) / sum(abs(txDataSyms).^2);
ds7 = ds6 / alpha7;
snr7 = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(ds7 - txDataSyms).^2));

fprintf('同步rho=%.4f\n', rho);
fprintf('v1 基线        : SNR=%.2f dB\n', snr1);
fprintf('v5 整体相位校正 : SNR=%.2f dB\n', snr5);
fprintf('v6 逐符号校正   : SNR=%.2f dB\n', snr6);
fprintf('v7 逐符号+整体  : SNR=%.2f dB\n', snr7);
fprintf('参考门槛 ≈ 3.3dB\n');
if snr7 > 3.3
    fprintf('>> 校正后过门槛! 把逐符号相位校正加入OFDM_RxMain即可。\n');
elseif snr7 > snr1 + 1
    fprintf('>> 校正有改善(%.1f dB)但不足, 需配合其他手段。\n', snr7 - snr1);
else
    fprintf('>> 相位校正无改善 -> 残余相位不是主因, 信号本身在-1.3dB。\n');
end
logResult('evm_phase', struct('v1',snr1,'v5',snr5,'v6',snr6,'v7',snr7));
