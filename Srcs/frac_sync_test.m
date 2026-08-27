%% frac_sync_test.m - 分数样点定时偏移检测与补偿 (假说: 采集错位0.x样点导致ICI)
% 方法: 1) syncLTE整数同步 2) 从导频子载波相位斜率估计分数偏移δ
%      3) 频域补偿 4) 重测EVM
% 若补偿后EVM显著改善 -> 分数定时是主因 -> 加入接收链 -> CRC有望!

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
NFFT = prm.NFFT;

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

%% 整数同步 + 解调
[rxAl, rho, ~, ~] = syncLTE(rxAvg, txIQ, prm);
rxGrid = OFDMDemod(rxAl, prm);

%% 估计分数定时偏移 δ: 导频子载波相位斜率
pilotIdx1 = prm.PilotSymbols(1)+1;
pilotIdx2 = prm.PilotSymbols(2)+1;
% 行->FFT bin索引m: 行601->bin1, ..., 行1200->bin600; 行1->bin1449, ..., 行600->bin2048
krow = (1:prm.NSubc).';
m = zeros(prm.NSubc,1);
m(krow>=601) = krow(krow>=601) - 600;
m(krow<601)  = krow(krow<601) + 1448;
clean = ~(krow>=451 & krow<=750);     % 排除污染区
clean = clean & krow>=201 & krow<=1000; % 再排除边缘弱区

phi1 = angle(rxGrid(clean, pilotIdx1) ./ rs0(clean));
phi2 = angle(rxGrid(clean, pilotIdx2) ./ rs1(clean));
% 展开相位
phi1 = unwrap(phi1); phi2 = unwrap(phi2);
mc = m(clean);
% 线性拟合: phi = -2*pi*m*delta/2048 + phi0  -> delta = -slope*2048/(2*pi)
slope1 = (mc - mean(mc)) \ (phi1 - mean(phi1));
slope2 = (mc - mean(mc)) \ (phi2 - mean(phi2));
delta1 = -slope1 * NFFT / (2*pi);
delta2 = -slope2 * NFFT / (2*pi);
fprintf('整数同步rho=%.4f\n', rho);
fprintf('分数定时估计: 导频1 delta=%.3f 样点, 导频2 delta=%.3f 样点\n', delta1, delta2);
delta = mean([delta1 delta2]);

%% 频域补偿: 乘 exp(+j*2*pi*m*delta/NFFT)
rxGridC = rxGrid .* exp(1j * 2*pi * m * delta / NFFT);

%% 分别计算补偿前后EVM (处理与evm_compare v1一致)
snr0 = calcEVMonGrid(rxGrid,  prm, rs0, rs1, txDataSyms);
snr1 = calcEVMonGrid(rxGridC, prm, rs0, rs1, txDataSyms);
fprintf('\n补偿前 EVM-SNR = %.2f dB\n', snr0);
fprintf('补偿后 EVM-SNR = %.2f dB (delta=%.3f样点)\n', snr1, delta);
if snr1 > snr0 + 1
    fprintf('>> 分数定时补偿有效(%.1f dB)! 加入接收链后CRC有望。\n', snr1 - snr0);
elseif snr1 > 3.3
    fprintf('>> 补偿后过门槛! 分数定时就是主因!\n');
else
    fprintf('>> 补偿改善有限 -> 分数定时不是主因。\n');
end
logResult('frac_sync_test', struct('delta1',delta1,'delta2',delta2,'snr0',snr0,'snr1',snr1));

%% ================= 函数 =================
function snr = calcEVMonGrid(rxGrid, prm, rs0, rs1, txDataSyms)
Nsc = prm.NSubc;
H = ChanEst(rxGrid, rs0, rs1, prm);
notchMask = false(Nsc,1); notchMask(451:750) = true;
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
err = dataSyms - txDataSyms;
snr = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(err).^2));
end
