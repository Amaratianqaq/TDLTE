function [bits, crcOK] = OFDM_RxMain(rxIQ, prm)
% OFDM_RxMain - 接收端总入口, 串联全部接收链路模块
% 输入:
%   rxIQ - 接收时域复数IQ信号
%   prm  - 参数结构体
% 输出:
%   bits  - 恢复的传输块bit列向量
%   crcOK - CRC校验是否通过 (logical)

end
