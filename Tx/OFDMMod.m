function txSignal = OFDMMod(grid, prm)
% OFDMMod - 发送端OFDM调制 (对应任务书: 产生频域数据 + 产生时域数据 + 添加CP)
% 处理步骤:
%   1. 子载波映射: NSubc(1200) -> NFFT(2048), 中间填0 (任务6: genFredata)
%   2. IFFT: 每个符号做2048点IFFT, 频域转时域 (任务7: genantdata)
%   3. 添加CP: 按LTE常规CP配置, 为每个OFDM符号添加循环前缀
% 输入:
%   grid - NSubc x NSym 资源网格 (1200 x 14)
%   prm  - 参数结构体 (含NFFT, NRB, CP配置等)
% 输出:
%   txSignal - 时域复数IQ信号 (1ms子帧, 30720样点)

end
