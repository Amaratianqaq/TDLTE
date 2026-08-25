%% tx_tone_scan.m - 单音频率扫描 (测绘天线耦合带宽)
% 逐个发送 2/3/5/7/9 MHz 单音(每次只发一个, 避免互调), 看哪些频率能回传。
% 目的: 判断天线耦合是"窄带谐振"(只通个别频率) 还是 "宽带"(OFDM才可能通)。
% 判定:
%   - 只有1~2个频率回来 -> 耦合窄带谐振 -> 调整天线姿态/距离, 直到多个频率都能回来
%   - 多个频率都能回来  -> 耦合宽带 -> OFDM应该能通, 问题在电平/饱和, 调增益衰减
% 前置: 天线保持当前姿态, 上位机设置不变。
% 使用: 直接运行, 约1分钟。

clear; clc;
addpath(fileparts(mfilename('fullpath')));   % 确保 logResult 可用

F0S = [2 3 5 7 9] * 1e6;    % 扫描频率
A   = 1200;                  % 单音幅度(60%满幅, 不削波)
SAMPLE_LENGTH = 30720;
Fs = 30.72e6;

B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';

fprintf('==== 单音频率扫描 (请勿碰设备/天线) ====\n');
fprintf('%-8s %-14s %-14s %-10s\n', '频率MHz', '峰/全局中位数', '峰/局部中位数', '判定');
for k = 1:numel(F0S)
    f0 = F0S(k);
    n = 0:SAMPLE_LENGTH-1;
    txI = A*cos(2*pi*f0*n/Fs); txQ = A*sin(2*pi*f0*n/Fs);
    txI(txI<0) = txI(txI<0)+4096; txQ(txQ<0) = txQ(txQ<0)+4096;
    txI = fix(txI); txQ = fix(txQ);
    len = SAMPLE_LENGTH*2;
    dataIQ = zeros(1,len);
    dataIQ(1,1:2:len-1) = txI;  dataIQ(1,2:2:len) = txQ;
    dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1).*16;
    dataIQ(1,2:2:len) = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
    dataIQ = uint16(dataIQ);

    z = toneCapture(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(z)
        fprintf('%-8d %-14s %-14s %-10s\n', f0/1e6, '未收到', '', '');
        continue;
    end
    N = numel(z);
    X = fft(z(:).', N);
    f = (0:N-1)/N*Fs; f(f>Fs/2) = f(f>Fs/2) - Fs;
    P = abs(X).^2;
    % --- 稳健噪声底: 全局中位数 vs 局部中位数(峰两侧±2MHz剔除峰区) ---
    mask = abs(f) > 0.2e6;
    globalMed = median(P(mask));
    peakBand  = abs(abs(f) - f0) < 0.3e6;                       % 峰区 ±0.3MHz
    localBand = abs(abs(f) - f0) < 2e6 & ~peakBand;             % 局部两侧
    localMed  = median(P(localBand));
    ownG = max(P(peakBand)) / globalMed;
    ownL = max(P(peakBand)) / max(localMed, eps);
    dcR  = max(P(abs(f) < 0.05e6)) / globalMed;
    hit  = ownL > 20;                                            % 用局部噪声底判定
    localRatios(k) = ownL;                                       % 记录用于日志
    fprintf('%-8.0f %-14.1f %-14.1f %-10s\n', f0/1e6, ownG, ownL, ternary(hit,'回传','无'));
    fprintf('         (全局中位数比值=%.1f, 局部中位数比值=%.1f, DC/全局=%.1f)\n', ownG, ownL, dcR);
    save(sprintf('tone_%dMHz.mat', f0/1e6), 'z', 'f0');
end

fprintf('\n>> 多个频率回传 -> 耦合宽带, 问题在电平/饱和(降增益);\n');
fprintf('   仅个别频率回传 -> 耦合窄带谐振, 调整天线姿态/距离让更多频率回来。\n');

%% 记录结果到日志
logResult('tx_tone_scan', struct('freqsMHz',F0S/1e6,'localRatios',localRatios));

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end

function z = toneCapture(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
z = [];
udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
try
    fopen(udp_obj);
    fwrite(udp_obj, cmdSync,   'uint8');
    fwrite(udp_obj, cmdRouter, 'uint8');
    fwrite(udp_obj, cmdDelay,  'uint8');
    fwrite(udp_obj, cmdTx,     'uint8');
    fwrite(udp_obj, cmdSend,   'uint8');
    SEND_PACKET_LENGTH = 512;
    for pn = 1:fix(SAMPLE_LENGTH*2/SEND_PACKET_LENGTH)
        fwrite(udp_obj, dataIQ(1,((pn-1)*SEND_PACKET_LENGTH+1):(pn*SEND_PACKET_LENGTH)), 'uint16');
    end
    fwrite(udp_obj, cmdGet, 'uint8');
    RECV_PACKET_LENGTH = 240*4;
    data_byte_numb = SAMPLE_LENGTH*4;
    buf = zeros(1, SAMPLE_LENGTH*4);
    data_byte_c = 0;
    recvInd = 1;
    while recvInd == 1
        [udp_data, count] = fread(udp_obj, RECV_PACKET_LENGTH);
        for pj = 1:count
            buf(data_byte_c+pj) = udp_data(pj);
        end
        data_byte_c = data_byte_c + count;
        if (data_byte_c >= data_byte_numb) || (data_byte_c == 0)
            recvInd = 0;
        end
    end
    fclose(udp_obj);
    if data_byte_c > 0
        N = floor(data_byte_c/4);
        b = double(buf(1:N*4));
        o = (0:N-1)*4;
        iw = b(o+1) + b(o+2)*256;
        qw = b(o+3) + b(o+4)*256;
        iv = floor(iw/16);  qv = floor(qw/16);
        iv(iv>2047) = iv(iv>2047) - 4096;
        qv(qv>2047) = qv(qv>2047) - 4096;
        z = iv + 1j*qv;
    end
catch
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
