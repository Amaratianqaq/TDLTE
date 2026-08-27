function test_offline_margin()
% test_offline_margin - 在保存的 K=128 平均数据上离线试解码优化 (无需硬件)
% 目标: 把每子载波SNR(1.80dB)与解码门槛(~4.5dB)之间 ~2.7dB 缺口用代码手段补:
%   V0 基线: OFDM_RxMain 原样 (NotchMHz=0, 全行噪声)
%   V1 噪声方差排除污染行451~750 (与subcarrierSNR一致)
%   V2 V1 + Notch LLR擦除 (NotchMHz=0.5/1.0/1.5/2.0)
%   V3 V1 + 每子载波LLR加权 (按各子载波实际噪声缩放)
%   V4 V3 + Notch1.0
% 输出: 各变体的 译码后BER 与 CRC (BER<2%且CRC=1即成功)
% 使用: 先跑过 rx_accum_v3 生成 rx_accum_v3_result.mat, 直接运行本脚本。

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

if ~exist('rx_accum_v3_result.mat', 'file')
    error('没有 rx_accum_v3_result.mat, 请先运行 rx_accum_v3');
end
S = load('rx_accum_v3_result.mat');
Davg = S.Davg(:).';
prm = GetLTEParams();

% 发送参考bits (rng42与rx_accum_v3一致)
rng(42);
[txBits, ~] = TBGen(prm);
txBits = txBits(:);

% 参考rho (仅显示)
rng(42);
txIQref = OFDM_TxMain(prm);
[~, rho, ~, ~] = syncLTE(Davg, txIQref, prm);
fprintf('平均后rho=%.4f (保存值%.4f)\n', rho, S.rhoAvg);

% ================= 复现收端链 (带旋钮) =================
rxGrid = OFDMDemod(Davg, prm);
[rs0, rs1] = RefSignalGen(prm);
H = ChanEst(rxGrid, rs0, rs1, prm);

notchRows = 451:750;                     % DC±2.25MHz 污染行
notchMask = false(prm.NSubc, 1); notchMask(notchRows) = true;
clean = ~notchMask;

% 平滑(污染行隔离)
for l = 1:prm.NSym
    h = H(:,l); h(notchMask) = NaN;
    h = movmean(h, 64, 'omitnan');
    h(notchMask) = H(notchMask,l);
    H(:,l) = h;
end

pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;

% 每子载波噪声方差 (导频残差, 平滑后H)
resK = 0.5*(abs(rxGrid(:,pilotIdx1) - H(:,pilotIdx1).*rs0).^2 + ...
            abs(rxGrid(:,pilotIdx2) - H(:,pilotIdx2).*rs1).^2);
noiseVarAll = median(resK);                 % 全行口径
noiseVarCln = median(resK(clean));          % 排除污染行口径
fprintf('噪声方差: 全行=%.4g | 排除污染行=%.4g (差 %.1f dB)\n', ...
    noiseVarAll, noiseVarCln, 10*log10(noiseVarAll/noiseVarCln));

% 均衡+解调 (用干净口径噪声, 与subcarrierSNR一致)
dataSyms = ExtractData(Equalize(rxGrid, H, 'MMSE', noiseVarCln), prm);
Nsc = prm.NSubc; NsymData = prm.NSym - prm.NPilot; Qm = prm.Qm;
rxLLR0 = DemodulateSoft(dataSyms, prm.Modulation, noiseVarCln);

% 发送端segInfo (用于码块去分段, rng42)
rng(42);
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[~, segInfo] = CodeBlockSeg(bitsCRC);

% ================= 变体解码 =================
    function [ber, crc] = runDecode(rxLLR, notchMHz)
        if notchMHz > 0
            Nnotch = round(notchMHz / 0.015);
            kNotch = (601 - Nnotch):(601 + Nnotch);
            kNotch = kNotch(kNotch >= 1 & kNotch <= Nsc);
            pos = (kNotch(:) - 1) * Qm;
            pos = pos + (0:Qm-1);
            pos = pos(:);
            posAll = pos + (0:NsymData-1) * Nsc * Qm;
            rxLLR(posAll(:)) = 0;
        end
        descrambled = DescrambleSoft(rxLLR, prm);
        G = Nsc * NsymData * prm.NL * Qm;
        C = segInfo.C; Kp = segInfo.Kp; Km = segInfo.Km; Cp = segInfo.Cp; Cm = segInfo.Cm;
        Krs = [repmat(Km, Cm, 1); repmat(Kp, Cp, 1)];
        cbDecoded = cell(1, C);
        ptr = 1;
        for r = 1:C
            E = prm.NL * Qm * floor(G / (prm.NL * Qm * C));
            cbBits = descrambled(ptr:ptr+E-1);
            ptr = ptr + E;
            rmOut = DeRateMatch(cbBits, Krs(r), C, G, Qm, prm.NL, prm.RV);
            [cbDecoded{r}, ~] = TurboDecode(rmOut, Krs(r), prm.maxIter);
        end
        [bitsOut, crc] = CodeBlockDeSeg(cbDecoded, segInfo);
        b = min(numel(bitsOut), numel(txBits));
        ber = mean(bitsOut(1:b) ~= txBits(1:b));
    end

fprintf('\n==== 解码变体 (数据: K=128平均) ====\n');
% V0: 基线
[~, crc0] = OFDM_RxMain(Davg, prm);
fprintf('V0 基线(全行噪声, Notch=0): CRC=%d\n', crc0);

% V1
[ber1, crc1] = runDecode(rxLLR0, 0);
fprintf('V1 排除污染行噪声      : BER=%.4f CRC=%d\n', ber1, crc1);

% V2
for nm = [0.5 1.0 1.5 2.0]
    [ber2, crc2] = runDecode(rxLLR0, nm);
    fprintf('V2 排除+Notch%.1fMHz    : BER=%.4f CRC=%d\n', nm, ber2, crc2);
end

% V3: 每子载波LLR加权
rxLLR3 = rxLLR0;
LLRm = reshape(rxLLR3, Nsc, NsymData*Qm);
w = noiseVarCln ./ (resK + 1e-9);
w = movmean(w, 32, 'omitnan');          % 平滑权重
LLRm = LLRm .* w;
rxLLR3 = max(min(LLRm(:), 30), -30);
[ber3, crc3] = runDecode(rxLLR3, 0);
fprintf('V3 排除+每子载波加权    : BER=%.4f CRC=%d\n', ber3, crc3);

% V4
[ber4, crc4] = runDecode(rxLLR3, 1.0);
fprintf('V4 排除+加权+Notch1.0M : BER=%.4f CRC=%d\n', ber4, crc4);

fprintf('\n参考: CRC=1需BER<约2%%; BER<5%%说明很近, 可再推。\n');

end
