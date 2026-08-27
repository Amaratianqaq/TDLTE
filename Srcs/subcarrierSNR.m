function snrPer = subcarrierSNR(rxAvg, txIQ, prm)
% subcarrierSNR - 均衡后每子载波SNR (决定性指标, 无需硬件)
% 输入:
%   rxAvg - 接收时域信号 (1x30720, 已同步/平均均可, 内部会再同步)
%   txIQ  - 发送参考时域信号 (1x30720, 与rx同帧)
%   prm   - 参数结构体 (GetLTEParams)
% 输出:
%   snrPer - 1200x1 每子载波SNR (dB, 对12个数据符号平均)
% 说明: 与 evm_check.m 同法; 噪声方差估计排除 DC±2.25MHz 污染行(451~750)。
%       用法: 先跑 rx_accum / rx_accum_v2 生成数据, 再离线分析。

[~, ~, ~, ~] = syncLTE(rxAvg, txIQ, prm);
rxGrid = OFDMDemod(rxAvg, prm);
[rs0, rs1] = RefSignalGen(prm);
H = ChanEst(rxGrid, rs0, rs1, prm);

% 污染区掩码 (与 OFDM_RxMain 口径一致: DC±2.25MHz + DC置空行)
notchMask = false(prm.NSubc, 1); notchMask(451:750) = true; notchMask(prm.DCIdx) = true;
for l = 1:prm.NSym
    h = H(:, l); h(notchMask) = NaN;
    h = movmean(h, 64, 'omitnan');
    h(notchMask) = H(notchMask, l);
    H(:, l) = h;
end

pilotIdx1 = prm.PilotSymbols(1) + 1; pilotIdx2 = prm.PilotSymbols(2) + 1;
clean = ~notchMask;
residual = [abs(rxGrid(clean,pilotIdx1) - H(clean,pilotIdx1).*rs0(clean)).^2; ...
            abs(rxGrid(clean,pilotIdx2) - H(clean,pilotIdx2).*rs1(clean)).^2];
noiseVar = max(median(residual), 1e-9);
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);
dataSyms = ExtractData(eqGrid, prm);

% 发送参考数据符号 (rng(42) 与发送端一致)
rng(42);
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[cbs, segInfo] = CodeBlockSeg(bitsCRC);
cbOutputs = cell(segInfo.C, 1);
for r = 1:segInfo.C
    coded = TurboEncode(cbs{r}, segInfo.K(r));
    cbOutputs{r} = RateMatch(coded, segInfo.K(r), segInfo.C, info.G, info.Qm, prm.NL, prm.RV, r);
end
codedBits = CodeBlockCat(cbOutputs);
scrambled = Scramble(codedBits, prm);
sym = Modulate(scrambled, prm.Modulation);
txGrid = ResMap(sym, rs0, rs1, prm);
txDataSyms = ExtractData(txGrid, prm);

err = dataSyms - txDataSyms;
Nsc = prm.NDataSubc; NsymData = prm.NSym - prm.NPilot;   % DC置空: 1199子载波
errM = reshape(err, Nsc, NsymData);
txM  = reshape(txDataSyms, Nsc, NsymData);
snrPer = 10*log10(mean(abs(txM).^2, 2) ./ (mean(abs(errM).^2, 2) + eps));
end
