function test_offline_margin3()
% test_offline_margin3 - 符号间相位漂移校正 + 信道IR统计 (无需硬件)
% 假设: 回环路径可能存在帧内残余频偏 -> 各OFDM符号相位不同 -> 均衡后星座散开。
% 方法: 用硬判决估计每个符号的残余相位, 校正后重新解调译码。
% 输出: 每符号残余相位序列 + 校正前后 BER/CRC + 信道冲激响应统计。

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

rng(42);
[txBits, ~] = TBGen(prm); txBits = txBits(:);

rxGrid = OFDMDemod(Davg, prm);
[rs0, rs1] = RefSignalGen(prm);
H = ChanEst(rxGrid, rs0, rs1, prm);

notchMask = false(prm.NSubc, 1); notchMask(451:750) = true;
clean = ~notchMask;
pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;

for l = 1:prm.NSym
    h = H(:,l); h(notchMask) = NaN;
    h = movmean(h, 64, 'omitnan');
    h(notchMask) = H(notchMask,l);
    H(:,l) = h;
end

resK = 0.5*(abs(rxGrid(:,pilotIdx1) - H(:,pilotIdx1).*rs0).^2 + ...
            abs(rxGrid(:,pilotIdx2) - H(:,pilotIdx2).*rs1).^2);
noiseVar = max(median(resK(clean)), 1e-9);
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
dataCols = [1:3 5:10 12:14];          % 数据符号列 (1-based, 非导频3/10)
dataSyms = ExtractData(eqGrid, prm);

% 每符号残余相位 (对导频列用已知参考, 对数据列用硬判决)
symPhase = zeros(prm.NSym, 1);
for l = 1:prm.NSym
    if ismember(l, [pilotIdx1 pilotIdx2])
        ref = (l == pilotIdx1) * rs0 + (l == pilotIdx2) * rs1;
        e = eqGrid(clean, l) - H(clean, l).*ref(clean);   % 误差(已均衡)
        symPhase(l) = angle(sum(conj(eqGrid(clean,l)) .* (H(clean,l).*ref(clean))));
    else
        d = eqGrid(clean, l);
        % 硬判决到最近星座点 (QPSK)
        hard = sign(real(d))/sqrt(2) + 1j*sign(imag(d))/sqrt(2);
        symPhase(l) = angle(sum(conj(d) .* hard));
    end
end
fprintf('每符号残余相位(度): %s\n', mat2str(round(rad2deg(symPhase), 1)'));

% 相位校正: 各符号数据乘 e^{-j*symPhase}
eqGridC = eqGrid;
for l = 1:prm.NSym
    eqGridC(:, l) = eqGrid(:, l) .* exp(-1j*symPhase(l));
end
dataSymsC = ExtractData(eqGridC, prm);

% 解码
rng(42);
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[~, segInfo] = CodeBlockSeg(bitsCRC);
Nsc = prm.NSubc; NsymData = prm.NSym - prm.NPilot; Qm = prm.Qm;

    function [ber, crc] = runDecode(syms)
        rxLLR = DemodulateSoft(syms, prm.Modulation, noiseVar);
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

[berA, crcA] = runDecode(dataSyms);
[berB, crcB] = runDecode(dataSymsC);
fprintf('\n校正前: BER=%.4f CRC=%d | 校正后: BER=%.4f CRC=%d\n', berA, crcA, berB, crcB);

% 信道IR统计 (导频列, 干净行)
Hp = H(:, pilotIdx1); Hp(~clean) = 0;
hIR = ifft(Hp, 1200);
p = abs(hIR).^2; ptot = sum(p);
CP_pts = round(144 / 32.55);
fprintf('\n信道IR: CP内能量=%.1f%% | CP外=%.1f%% | 最强径=样点%d (%.2fus)\n', ...
    100*sum(p(1:CP_pts))/ptot, 100*sum(p(CP_pts+1:end))/ptot, find(p==max(p),1), find(p==max(p),1)*0.0556);
[pm, im] = max(p(CP_pts+1:end));
fprintf('CP外最强径: %.1fus, 相对主径 %.1f dB\n', (CP_pts+im)*0.0556, 10*log10(pm/max(p)+eps));

end
