function txIQ = OFDM_TxMain(prm)
% OFDM_TxMain - 发送端总入口, 串联全部发送链路模块
% 处理链路 (按调用顺序):
%   TBGen(信源) -> CRCAttach(加CRC) -> CodeBlockSeg(码块分割)
%   -> TurboEncode(Turbo编码) -> RateMatch(速率匹配, 含子块交织)
%   -> CodeBlockCat(码块级联) -> Scramble(加扰) -> Modulate(调制)
%   -> RefSignalGen(生成导频) -> ResMap(资源映射) -> OFDMMod(OFDM调制)
%   -> txIQ
% 输入:
%   prm - 参数结构体 (由 GetLTEParams 返回)
% 输出:
%   txIQ - 时域复数IQ信号 (1ms子帧, 30720样点)

%% 1) 信源 -> 添加TB级CRC24A
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');

%% 2) 码块分割 -> Turbo编码 -> 速率匹配 -> 码块级联
[cbs, segInfo] = CodeBlockSeg(bitsCRC);
cbOutputs = cell(segInfo.C, 1);
for r = 1:segInfo.C
    coded = TurboEncode(cbs{r}, segInfo.K(r));
    cbOutputs{r} = RateMatch(coded, segInfo.K(r), segInfo.C, ...
                             info.G, info.Qm, prm.NL, prm.RV);
end
codedBits = CodeBlockCat(cbOutputs);               % 长度 = G

%% 3) 加扰 -> 调制 (G/Qm = 1200*12 个复数符号)
scrambled = Scramble(codedBits, prm);
sym = Modulate(scrambled, prm.Modulation);

%% 4) 生成导频 -> 资源映射 -> OFDM调制 (子载波映射+IFFT+加CP)
[rs0, rs1] = RefSignalGen(prm);
grid = ResMap(sym, rs0, rs1, prm);                 % 1200 x 14
txIQ = OFDMMod(grid, prm);                         % 1 x 30720
end
