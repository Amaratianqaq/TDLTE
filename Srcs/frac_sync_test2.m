%% frac_sync_test2.m - 分数定时: 时域重采样补偿 (修正版)
% 之前的频域相位补偿只能修子载波旋转(均衡也能做), 修不了ICI。
% ICI只能通过"时域分数样点重采样"消除: 把信号平移δ个样点再FFT。
% 方法: 1) syncLTE整数对齐 2) xcorr峰抛物线插值估计分数δ
%      3) FFT域平移(时域重采样) 4) 重测EVM (正/负两个方向取优)

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
N = numel(rxAvg); NFFT = prm.NFFT;

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

%% 整数同步
[rxAl, rho, ~, ~] = syncLTE(rxAvg, txIQ, prm);

%% 分数δ估计1: xcorr峰抛物线插值
c = xcorr(rxAl, txIQ);
[cmax, imax] = max(abs(c));
d0 = abs(c(imax-1)); d1 = cmax; d2 = abs(c(imax+1));
frac1 = 0.5*(d0 - d2)/(d0 - 2*d1 + d2 + eps);
% 估计2: 导频相位斜率 (小数部分)
rxGrid0 = OFDMDemod(rxAl, prm);
pilotIdx1 = prm.PilotSymbols(1)+1;
krow = (1:prm.NSubc).';
m = zeros(prm.NSubc,1);
m(krow>=601) = krow(krow>=601)-600; m(krow<601) = krow(krow<601)+1448;
clean = ~(krow>=451 & krow<=750) & krow>=201 & krow<=1000;
phi = unwrap(angle(rxGrid0(clean,pilotIdx1)./rs0(clean)));
slope = (m(clean)-mean(m(clean))) \ (phi-mean(phi));
frac2 = -slope*NFFT/(2*pi);
frac2 = frac2 - round(frac2);   % 取小数部分
fprintf('分数δ: xcorr抛物线=%.3f, 导频相位=%.3f 样点\n', frac1, frac2);
delta = frac1;   % 用xcorr抛物线(更直接)

%% 时域重采样: 平移delta样点 (正负都试)
F = exp(1j*2*pi*(0:N-1)*delta/N);
rxShift1 = ifft(fft(rxAl) .* F);            % 平移+delta
rxShift2 = ifft(fft(rxAl) .* conj(F));      % 平移-delta

snr0 = calcEVMonSig(rxAl,      prm, rs0, rs1, txDataSyms);
snr1 = calcEVMonSig(rxShift1,  prm, rs0, rs1, txDataSyms);
snr2 = calcEVMonSig(rxShift2,  prm, rs0, rs1, txDataSyms);
fprintf('\n原始      EVM-SNR = %.2f dB\n', snr0);
fprintf('重采样+δ  EVM-SNR = %.2f dB\n', snr1);
fprintf('重采样-δ  EVM-SNR = %.2f dB\n', snr2);
best = max([snr1 snr2]);
if best > snr0 + 1
    fprintf('>> 时域重采样有效(%.1f dB)! 分数定时确认 -> 加入接收链!\n', best-snr0);
else
    fprintf('>> 重采样改善有限 -> 分数定时不是主因。\n');
end
logResult('frac_sync_test2', struct('frac1',frac1,'frac2',frac2,'snr0',snr0,'snrP',snr1,'snrM',snr2));

%% ================= 函数 =================
function snr = calcEVMonSig(rxSig, prm, rs0, rs1, txDataSyms)
rxGrid = OFDMDemod(rxSig, prm);
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
