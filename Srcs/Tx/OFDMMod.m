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

NFFT  = prm.NFFT;                   % 2048
NSubc = prm.NSubc;                  % 1200
NSym  = prm.NSym;                   % 14
NZero = NFFT - NSubc;               % 中间补零个数 = 848 (任务6)

%% 1) 子载波映射: NSubc(1200) -> NFFT(2048) (任务6: genFredata)
% 任务书: 每个符号取"后端600数据"作新数组前端, "前端600数据"作新数组后端,
% 中间填充848个0 -> 一行 14x2048 频域数据
% 即 freqData(l,:) = [ x(601:1200), zeros(1,848), x(1:600) ] (x为该符号1200个子载波)
freqData = zeros(NSym, NFFT);
for l = 1:NSym
    x = grid(:, l);                                % 该符号的1200个子载波 (列向量)
    freqData(l, :) = [x(NSubc/2+1:NSubc).', zeros(1, NZero), x(1:NSubc/2).'];
end

%% 2) IFFT: 频域转时域 (任务7: genantdata)
% 每个符号用MATLAB库函数ifft做2048点ifft, 得到 14x2048 时域数据
timeData = ifft(freqData, NFFT, 2);                % 14 x 2048

%% 3) 串并转换 + 添加CP (文档2.2.13, prm.CPLengths)
% 常规CP: 每时隙符号0的CP=160, 其余符号CP=144 -> 每子帧CP共2048点,
% 总样点数 = 14*2048 + 2048 = 30720 (与 OFDM_RFLoopback 的 SAMPLE_LENGTH 一致)
txSignal = zeros(1, prm.NSample);
ptr = 1;
for l = 1:NSym
    cpLen = prm.CPLengths(l);                      % 本符号CP长度 (160/144)
    body  = timeData(l, :);                        % 该符号2048个时域样点
    txSignal(ptr : ptr + cpLen - 1) = body(NFFT - cpLen + 1 : NFFT);  % CP = 尾部拷贝
    ptr = ptr + cpLen;
    txSignal(ptr : ptr + NFFT - 1)  = body;        % 符号体
    ptr = ptr + NFFT;
end
end
