function [bits, crcOK] = OFDM_RxMain(rxIQ, prm)
% OFDM_RxMain - 接收端总入口, 串联全部接收链路模块
% 处理链路 (按调用顺序):
%   rxIQ -> OFDMDemod(去CP+FFT) -> ChanEst(信道估计) -> Equalize(MMSE均衡)
%        -> ExtractData(提取数据) -> DemodulateSoft(软判决→LLR) -> DescrambleSoft(解扰)
%        -> DeRateMatch(解速率匹配, LLR域) -> TurboDecode(Turbo译码)
%        -> CodeBlockDeSeg(码块去分段+解CRC) -> bits, crcOK
% 注: 相比旧硬判决链, 软判决+MMSE+更多Turbo迭代可带来2~4dB增益 (弱信号场景)。
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

%% 4) 均衡: MMSE (噪声方差由导频残余估计)
pilotIdx1 = prm.PilotSymbols(1) + 1;
pilotIdx2 = prm.PilotSymbols(2) + 1;
residual = [abs(rxGrid(:,pilotIdx1) - H(:,pilotIdx1).*rs0).^2; ...
            abs(rxGrid(:,pilotIdx2) - H(:,pilotIdx2).*rs1).^2];
noiseVar = max(mean(residual), 1e-9);     % 防除零
eqGrid = Equalize(rxGrid, H, 'MMSE', noiseVar);

%% 5) 提取数据符号: 去除导频列
dataSyms = ExtractData(eqGrid, prm);

%% 6) 软判决解调: 输出比特LLR (max-log)
rxLLR = DemodulateSoft(dataSyms, prm.Modulation, noiseVar);

%% 6.5) 频域去干扰: 置零低频子载波LLR (设备0~1MHz确定性干扰, 按擦除处理)
% DC位于网格行601; 子载波间隔15kHz。置零 |f|<=NotchMHz 的行对应的LLR。
if isfield(prm, 'NotchMHz') && prm.NotchMHz > 0
    Nsc = prm.NSubc;  Qm = prm.Qm;
    NsymData = prm.NSym - prm.NPilot;           % 数据符号数 12
    Nnotch = round(prm.NotchMHz / 0.015);       % 子载波数
    kNotch = (601 - Nnotch) : (601 + Nnotch);
    kNotch = kNotch(kNotch >= 1 & kNotch <= Nsc);
    pos = (kNotch(:) - 1) * Qm;                 % 子载波k在符号内的基位置
    pos = pos + (0:Qm-1);                        % 该子载波的Qm个bit
    pos = pos(:);
    posAll = pos + (0:NsymData-1) * Nsc * Qm;    % 扩展到12个数据符号
    rxLLR(posAll(:)) = 0;                        % 擦除(0=无信息)
end

%% 7) 解扰 (LLR域: 乘±1)
descrambled = DescrambleSoft(rxLLR, prm);

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
