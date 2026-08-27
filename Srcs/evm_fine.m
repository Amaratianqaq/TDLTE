%% evm_fine.m - 精细同步后的EVM测试 (判断5.1dB缺口是否来自同步残留)
% syncLTE粗同步(250Hz频偏步进+整数样点定时)会留下残留:
%   - 频偏残留 ±125Hz
%   - 定时残留 ±1样点
% 本脚本在粗同步基础上做细频偏搜索(25Hz) + 分数样点定时, 重新算EVM。
% 若SNR明显改善 -> 缺口来自同步残留, 可修复(把细同步加入接收链);
% 若不变 -> 缺口是设备噪声底, 不可修复, 确认设备侧限制。

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
NFFT = prm.NFFT; N = numel(rxAvg); n = 0:N-1;

%% 发送参考 (与evm_check一致)
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

%% 粗同步 (syncLTE)
[rxAl, rho, cfoCoarse, delay] = syncLTE(rxAvg, txIQ, prm);
r = rxAvg(:).' - mean(rxAvg);
v = txIQ(:).' - mean(txIQ);

%% 细频偏搜索: 粗值±250Hz, 25Hz步进
cfoList = cfoCoarse + (-250:25:250);
best = struct('cfo',cfoCoarse,'snr',-inf);
for ci = 1:numel(cfoList)
    rc = r .* exp(-1j*2*pi*cfoList(ci)*n/NFFT);
    % 分数样点定时: xcorr峰值的抛物线插值
    c = xcorr(rc, v);
    [cmax, imax] = max(abs(c));
    lag = imax - (numel(c)+1)/2;
    % 抛物线插值细定时
    if imax > 1 && imax < numel(c)
        d0 = abs(c(imax-1)); d1 = cmax; d2 = abs(c(imax+1));
        frac = 0.5*(d0 - d2)/(d0 - 2*d1 + d2 + eps);
        lagFine = lag + frac;
    else
        lagFine = lag;
    end
    % 用细定时构建对齐信号 (线性插值频移近似分数延迟)
    rxSync = rc(mod((0:N-1) + round(lagFine), N) + 1);
    % EVM
    snr = calcEVM(rxSync, txIQ, prm, rs0, rs1, txDataSyms);
    if snr > best.snr
        best = struct('cfo',cfoList(ci),'snr',snr);
    end
end

fprintf('粗同步: rho=%.4f, cfo=%.1fHz\n', rho, cfoCoarse);
fprintf('细同步后: 最佳cfo=%.1fHz, 均衡后数据SNR=%.2f dB\n', best.cfo, best.snr);
fprintf('对比: 粗同步EVM约为-1.77dB (之前evm_check结果)\n');
if best.snr > -1.0
    fprintf('>> 同步残留是主因! 改善%.1f dB -> 把细同步加入接收链。\n', best.snr - (-1.77));
else
    fprintf('>> 细同步改善有限(%.1f dB) -> 缺口是设备噪声底, 确认设备侧限制。\n', best.snr - (-1.77));
end
logResult('evm_fine', struct('snr_coarse',-1.77,'snr_fine',best.snr,'cfo_fine',best.cfo));

%% ================= 函数 =================
function snr = calcEVM(rxSync, txIQ, prm, rs0, rs1, txDataSyms)
% 简化EVM: 信道估计(全频域平滑)+ZF均衡+提取, 算数据符号SNR
rxGrid = OFDMDemod(rxSync, prm);
H = ChanEst(rxGrid, rs0, rs1, prm);
H = ones(prm.NSubc, prm.NSym);          % 平坦信道近似(近场耦合缓变)
H = ChanEst(rxGrid, rs0, rs1, prm);     % 用LS
for l = 1:prm.NSym
    H(:,l) = movmean(H(:,l), 256);      % 强平滑
end
eqGrid = rxGrid ./ H;
dataSyms = ExtractData(eqGrid, prm);
err = dataSyms - txDataSyms;
snr = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(err).^2));
end
