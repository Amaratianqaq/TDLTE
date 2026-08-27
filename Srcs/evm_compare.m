%% evm_compare.m - 统一EVM测量: 不同信道估计处理对比 (无需硬件)
% 用已保存的 rx_accum_result.mat (当前设置, K=16平均), 测4种处理的EVM与CRC:
%   v1 当前链: chanSmooth=64 + 污染区隔离
%   v2 强平滑 : chanSmooth=256 + 污染区隔离
%   v3 强平滑 : chanSmooth=256, 无隔离
%   v4 理想H  : H=1 (参考上限)
% 若某变体EVM>3.3dB或CRC=1 -> 信道估计是瓶颈, 把该处理写入OFDM_RxMain。

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

%% 发送参考数据符号
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

%% 同步
[rxAl, rho, ~, ~] = syncLTE(rxAvg, txIQ, prm);
fprintf('同步rho=%.4f\n', rho);

%% 4种处理
variants = {
    'v1 平滑64+隔离', 64,  true;
    'v2 平滑256+隔离', 256, true;
    'v3 平滑256无隔离', 256, false;
    'v4 理想H',        0,   false;
    };
fprintf('\n%-18s %-10s %-8s\n', '处理', 'SNR(dB)', 'CRC');
for vi = 1:size(variants,1)
    name = variants{vi,1}; sm = variants{vi,2}; iso = variants{vi,3};
    rxGrid = OFDMDemod(rxAl, prm);
    if sm == 0
        H = ones(prm.NSubc, prm.NSym);      % 理想H
    else
        H = ChanEst(rxGrid, rs0, rs1, prm);
        if iso
            % 污染区隔离: 用实测污染区行451~750 (DC±2.25MHz)
            notchMask = false(prm.NSubc,1);
            kNotch = 451:750;
            kNotch = kNotch(kNotch>=1 & kNotch<=prm.NSubc);
            notchMask(kNotch) = true;
        else
            notchMask = false(prm.NSubc,1);
        end
        for l = 1:prm.NSym
            h = H(:,l); h(notchMask) = NaN;
            h = movmean(h, sm, 'omitnan');
            h(notchMask) = H(notchMask,l);
            H(:,l) = h;
        end
    end
    % 噪声方差
    pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;
    if sm == 0
        noiseVar = 1e-9;
    else
        if iso
            clean = ~notchMask;
        else
            clean = true(prm.NSubc,1);
        end
        residual = [abs(rxGrid(clean,pilotIdx1)-H(clean,pilotIdx1).*rs0(clean)).^2; ...
                    abs(rxGrid(clean,pilotIdx2)-H(clean,pilotIdx2).*rs1(clean)).^2];
        noiseVar = max(median(residual), 1e-9);
    end
    eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
    dataSyms = ExtractData(eqGrid, prm);
    err = dataSyms - txDataSyms;
    snr = 10*log10(mean(abs(txDataSyms).^2) / mean(abs(err).^2));
    % CRC
    crc = NaN;
    try
        p2 = GetLTEParams();
        if sm == 0, p2.idealChan = true; p2.noiseVarOverride = 1e-9; end
        if sm == 256 && iso, p2.chanSmooth = 256; p2.NotchMHz = 2.25; end
        if sm == 256 && ~iso, p2.chanSmooth = 256; p2.NotchMHz = 0; end
        if sm == 64, p2.chanSmooth = 64; p2.NotchMHz = 0; end
        [~, crc] = OFDM_RxMain(rxAl, p2);
    catch
    end
    fprintf('%-18s %-10.2f %-8d\n', name, snr, crc);
    logResult('evm_compare', struct('variant',name,'snr',snr,'crc',crc));
end
fprintf('\n>> 找SNR>3.3dB或CRC=1的变体, 若存在则把对应处理写入OFDM_RxMain。\n');
