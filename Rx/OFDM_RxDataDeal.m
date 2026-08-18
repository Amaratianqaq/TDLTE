function [rxI, rxQ] = OFDM_RxDataDeal(rawData, freqDiv, dataByteNumb)
% OFDM_RxDataDeal - 接收原始字节数据解析为IQ两路
% 被 OFDM_RFLoopback.m 调用
% 输入:
%   rawData      - 接收的原始字节数组
%   freqDiv      - 分频系数
%   dataByteNumb - 接收数据总字节数
% 输出:
%   rxI - 接收I路数据
%   rxQ - 接收Q路数据

end
