function [bits, crcOK] = OFDM_RxMain(rxIQ, prm)
% OFDM_RxMain - 接收端总入口, 串联全部接收链路模块
% 处理链路 (按调用顺序):
%   rxIQ -> OFDMDemod(去CP+FFT) -> ChanEst(信道估计) -> Equalize(均衡)
%        -> ExtractData(提取数据) -> Demodulate(解调) -> Descramble(解扰)
%        -> DeRateMatch(解速率匹配) -> TurboDecode(Turbo译码)
%        -> CodeBlockDeSeg(码块去分段+解CRC) -> bits, crcOK
% 输入:
%   rxIQ - 接收时域复数IQ信号 (1ms子帧, 30720样点)
%   prm  - 参数结构体 (由GetLTEParams返回)
% 输出:
%   bits  - 恢复的传输块bit列向量
%   crcOK - CRC校验是否通过 (logical)

%% 1) OFDM解调: 去CP + FFT + 子载波提取
rxGrid = OFDMDemod(rxIQ, prm);

%% 2) 生成本地参考信号 (接收端独立生成, 与发送端一致)
[rs0, rs1] = RefSignalGen(prm);

%% 3) 信道估计: 基于导频的LS估计 + 插值
H = ChanEst(rxGrid, rs0, rs1, prm);

%% 4) 均衡: ZF方法
eqGrid = Equalize(rxGrid, H, 'ZF');

%% 5) 提取数据符号: 去除导频列
dataSyms = ExtractData(eqGrid, prm);

%% 6) 解调: 硬判决
rxBits = Demodulate(dataSyms, prm.Modulation);

%% 7) 解扰
descrambled = Descramble(rxBits, prm);

%% 8) 码块分割信息 (接收端独立重算, 与发送端一致)
tbBits = CRCAttach(zeros(prm.TBsize, 1), '24A');  % 假数据, 仅获取info
[~, segInfo] = CodeBlockSeg(tbBits);
C  = segInfo.C;
Kp = segInfo.Kp;
Km = segInfo.Km;
Cp = segInfo.Cp;
Cm = segInfo.Cm;

% 总可用bit数
G = prm.NSubc * (prm.NSym - prm.NPilot) * prm.NL * prm.Qm;

% 块长度序列
Krs = [repmat(Km, Cm, 1); repmat(Kp, Cp, 1)];

%% 9~10) 逐块: 解速率匹配 -> Turbo译码
cbDecoded = cell(1, C);
ptr = 1;
for r = 1:C
    Kr = Krs(r);

    % 计算本码块速率匹配输出长度
    E = prm.NL * prm.Qm * floor(G / (prm.NL * prm.Qm * C));

    % 提取本码块的接收bit
    cbBits = descrambled(ptr:ptr+E-1);
    ptr = ptr + E;

    % 解速率匹配
    rmOut = DeRateMatch(cbBits, Kr, C, G, prm.Qm, prm.NL, prm.RV);

    % Turbo译码
    [cbDecoded{r}, ~] = TurboDecode(rmOut, Kr, prm.maxIter);
end

%% 11) 码块去分段 + TB-CRC校验
[bits, crcOK] = CodeBlockDeSeg(cbDecoded, segInfo);

end
