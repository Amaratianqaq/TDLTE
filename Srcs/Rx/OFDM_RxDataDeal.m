function [rxI, rxQ] = OFDM_RxDataDeal(rawData, freqDiv, dataByteNumb)
% OFDM_RxDataDeal - 接收原始字节数据解析为IQ两路
% 被 OFDM_RFLoopback.m 调用 (第112行)
% 输入:
%   rawData      - 接收的原始字节数组 (uint8)
%   freqDiv      - 分频系数 (0=3字节/样点, 1=4字节/样点)
%   dataByteNumb - 接收数据总字节数
% 输出:
%   rxI - 接收I路数据 (1×30720)
%   rxQ - 接收Q路数据 (1×30720)
%
% 解包逻辑 (与发送端OFDM_RFLoopback.m第34~37行对称):
%   发送端打包:
%     I路: x = x * 16  (12bit左移4位, 低4位补0)
%     Q路: x = b7~b0 空4bits b11~b8 (字节重排)
%   接收端解包:
%     freqDiv=1: 每样点4字节 (I路2字节 + Q路2字节)
%     freqDiv=0: 每样点3字节 (I路12bit + Q路12bit)
%     I路: 右移4位, 取高12位
%     Q路: 反向字节重排, 恢复原始12位

SAMPLE_LENGTH = 30720;

%% 根据分频系数确定每样点字节数
if freqDiv == 0
    bytesPerSample = 3;
else
    bytesPerSample = 4;
end

numSamples = min(SAMPLE_LENGTH, floor(dataByteNumb / bytesPerSample));
rxI = zeros(1, SAMPLE_LENGTH);
rxQ = zeros(1, SAMPLE_LENGTH);

for n = 1:numSamples
    idx = (n-1) * bytesPerSample + 1;

    if freqDiv == 0
        %% 3字节模式: 24bit = I路12bit + Q路12bit
        b0 = rawData(idx);
        b1 = rawData(idx + 1);
        b2 = rawData(idx + 2);

        % I路: 高12位 (b0的8位 + b1的高4位)
        i_raw = bitshift(b0, 4) + bitshift(b1, -4);

        % Q路: 低12位 (b1的低4位 + b2的8位)
        q_raw = bitand(b1, 15) * 256 + b2;

    else
        %% 4字节模式: I路2字节 + Q路2字节
        i_lo = rawData(idx);
        i_hi = rawData(idx + 1);
        i_raw = i_lo + i_hi * 256;  % 小端序组成uint16

        q_lo = rawData(idx + 2);
        q_hi = rawData(idx + 3);
        q_raw = q_lo + q_hi * 256;  % 小端序组成uint16

        % I路解包: 右移4位, 取高12位
        i_raw = bitshift(i_raw, -4);

        % Q路解包: 反向字节重排
        % 发送端: Q路 = 高字节b7~b0, 低字节[空4bits b11~b8]
        % 接收端还原: 提取高8位和低字节的低4位
        q_raw = bitshift(q_raw, -8) + bitand(q_raw, 15) * 256;
    end

    %% 处理负数的补码还原 (12bit: 值 > 2047 时减 4096)
    if i_raw > 2047
        i_raw = i_raw - 4096;
    end
    if q_raw > 2047
        q_raw = q_raw - 4096;
    end

    rxI(n) = i_raw;
    rxQ(n) = q_raw;
end

end
