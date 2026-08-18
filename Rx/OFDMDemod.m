function grid = OFDMDemod(rxSignal, prm)
% OFDMDemod - 接收端OFDM解调 (对应任务书: 去CP + 时域转频域 + 解资源映射)
% 处理步骤:
%   1. 去CP: 按LTE常规CP配置, 去除每个OFDM符号的循环前缀
%   2. FFT: 每个符号做2048点FFT, 时域转频域 (任务8: OFDM_demod)
%   3. 子载波提取: NFFT(2048) -> NSubc(1200), 去除填0部分
% 输入:
%   rxSignal - 时域复数IQ信号 (1ms子帧, 30720样点)
%   prm      - 参数结构体 (含NFFT, NRB, CP配置等)
% 输出:
%   grid - NSubc x NSym 接收资源网格 (1200 x 14)

end
