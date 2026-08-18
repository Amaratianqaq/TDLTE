function [bits, crcOK] = OFDM_RxMain(rxIQ, prm)
% OFDM_RxMain - 接收端总入口, 串联全部接收链路模块
% 处理链路 (按调用顺序):
%   rxIQ -> OFDMDemod(去CP+FFT) -> ChanEst(信道估计) -> Equalize(均衡)
%        -> ExtractData(提取数据) -> Demodulate(解调) -> Descramble(解扰)
%        -> DeRateMatch(解速率匹配) -> TurboDecode(Turbo译码)
%        -> CodeBlockDeSeg(码块去分段+解CRC) -> bits, crcOK
% 输入:
%   rxIQ - 接收时域复数IQ信号
%   prm  - 参数结构体
% 输出:
%   bits  - 恢复的传输块bit列向量
%   crcOK - CRC校验是否通过 (logical)

end
