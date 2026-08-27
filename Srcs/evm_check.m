%% evm_check.m - 实测均衡后每子载波SNR (决定性指标, 无需硬件)
% 用已保存的 rx_accum_result.mat (多帧平均) 计算真实数据符号EVM/SNR:
%   SNR = 10*log10(信号功率/误差功率)
% 这直接给出"离可解调还差几dB", 不再用rho间接估计。
% 使用: 先跑过 rx_accum 生成数据, 再运行本脚本。

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

%% 1) 发送端参考数据符号 (与rx_accum发送的完全一致)
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
txDataSyms = ExtractData(txGrid, prm);          % 参考数据符号 (与rx同序)

%% 2) 接收端: 同步 → 解调 → 信道估计(平滑,污染隔离) → MMSE → 提取
[rxAl, rho, ~, ~] = syncLTE(rxAvg, txIQ, prm);
rxGrid = OFDMDemod(rxAl, prm);

% 污染区掩码 (与OFDM_RxMain一致)
if isfield(prm,'NotchMHz') && prm.NotchMHz > 0
    Nnotch = round(prm.NotchMHz/0.015);
    kNotch = (601-Nnotch):(601+Nnotch);
    kNotch = kNotch(kNotch>=1 & kNotch<=prm.NSubc);
    notchMask = false(prm.NSubc,1); notchMask(kNotch) = true;
else
    notchMask = false(prm.NSubc,1);
end
cleanRows = ~notchMask;

H = ChanEst(rxGrid, rs0, rs1, prm);
if isfield(prm,'chanSmooth') && prm.chanSmooth > 1
    for l = 1:prm.NSym
        h = H(:,l); h(notchMask) = NaN;
        h = movmean(h, prm.chanSmooth, 'omitnan');
        h(notchMask) = H(notchMask,l);
        H(:,l) = h;
    end
end
pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;
residual = [abs(rxGrid(cleanRows,pilotIdx1)-H(cleanRows,pilotIdx1).*rs0(cleanRows)).^2; ...
            abs(rxGrid(cleanRows,pilotIdx2)-H(cleanRows,pilotIdx2).*rs1(cleanRows)).^2];
noiseVar = max(median(residual), 1e-9);
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
rxDataSyms = ExtractData(eqGrid, prm);

%% 3) EVM/SNR
err = rxDataSyms - txDataSyms;
SNRall = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(err).^2));
% 排除污染区的SNR
Nsc = prm.NSubc; Qm = prm.Qm; NsymData = prm.NSym - prm.NPilot;
idxAll = reshape(1:numel(txDataSyms), Nsc, NsymData);   % 行=子载波, 列=符号
cleanIdx = idxAll(cleanRows, :);
errC = err(cleanIdx(:));  txC = txDataSyms(cleanIdx(:));
SNRclean = 10*log10(mean(abs(txC).^2) / mean(abs(errC).^2));

fprintf('同步rho=%.4f\n', rho);
fprintf('均衡后数据符号 SNR (全部子载波) = %.2f dB\n', SNRall);
fprintf('均衡后数据符号 SNR (排除污染区) = %.2f dB\n', SNRclean);
fprintf('参考: 离线门槛 ≈ 3.3dB/子载波\n');
if SNRclean > 3.5
    fprintf('>> 已过门槛! 理论上应能解出, 检查Turbo/其它环节。\n');
elseif SNRclean > 0
    fprintf('>> 距门槛还差 %.1f dB -> 可再推(更大K/微调)。\n', 3.3 - SNRclean);
else
    fprintf('>> 距门槛还差 %.1f dB -> 信号强度不足, 设备接收路径限制。\n', 3.3 - SNRclean);
end
logResult('evm_check', struct('SNR_all',SNRall,'SNR_clean',SNRclean,'rho',rho));
