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

NFFT  = prm.NFFT;                   % 2048
NSubc = prm.NSubc;                  % 1200
NSym  = prm.NSym;                   % 14

%% 1) 去CP: 按CP长度表去除每个符号的循环前缀
% 预分配: 每符号2048个样点, 共14个符号
timeData = zeros(NSym, NFFT);
ptr = 1;
for l = 1:NSym
    cpLen = prm.CPLengths(l);                  % 本符号CP长度 (160/144)
    ptr = ptr + cpLen;                         % 跳过CP
    timeData(l, :) = rxSignal(ptr : ptr + NFFT - 1);  % 提取符号体
    ptr = ptr + NFFT;
end

%% 2) FFT: 时域转频域
freqData = fft(timeData, NFFT, 2);            % 14 x 2048

%% 3) 子载波提取: NFFT(2048) -> NSubc(1200)
% 与OFDMMod的映射严格对称:
%   FFT输出的前600个子载波 -> 网格的后600行
%   FFT输出的后600个子载波 -> 网格的前600行
grid = zeros(NSubc, NSym);
for l = 1:NSym
    x = freqData(l, :);
    grid(1:NSubc/2, l)          = x(NFFT - NSubc/2 + 1 : NFFT).';   % 后600 -> 前600
    grid(NSubc/2+1:NSubc, l)    = x(1:NSubc/2).';                    % 前600 -> 后600
end

end
