%% tone_quick.m - 单音回传快速检测 (对照实验用)
% 每次发送5MHz单音一次, 输出: 5MHz峰/局部噪声底、DC、rx功率, 一行搞定。
% 用途: 配合对照实验表(拔天线/拉距离/改频点/改衰减), 每个状态跑一次记录输出。
% 使用: 直接运行。

clear; clc;
addpath(fileparts(mfilename('fullpath')));   % 确保 logResult 可用

f0 = 5e6;  A = 1200;
SAMPLE_LENGTH = 30720;  Fs = 30.72e6;

B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';

% 5MHz单音
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
    fprintf('未收到数据\n');
    return;
end
N = numel(z);
X = fft(z(:).', N);
f = (0:N-1)/N*Fs; f(f>Fs/2) = f(f>Fs/2) - Fs;
P = abs(X).^2;
mask = abs(f) > 0.2e6;
globalMed = median(P(mask));
peakBand = abs(abs(f)-f0) < 0.3e6;
localBand = abs(abs(f)-f0) < 2e6 & ~peakBand;
localMed = median(P(localBand));
ownL = max(P(peakBand)) / max(localMed, eps);
dcR  = max(P(abs(f) < 0.05e6)) / globalMed;
rxPow = mean(abs(z).^2);
fprintf('5MHz峰/局部噪声底=%9.1fx | DC/全局=%8.1fx | rx功率=%.3f\n', ownL, dcR, rxPow);

%% 记录结果到日志
logResult('tone_quick', struct('peak5M',ownL,'dcRatio',dcR,'rxpow',rxPow));

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
        iv = bitshift(iw,-8) + bitand(iw,15)*256;
        qv = bitshift(qw,-8) + bitand(qw,15)*256;
        iv(iv>2047) = iv(iv>2047) - 4096;
        qv(qv>2047) = qv(qv>2047) - 4096;
        z = iv + 1j*qv;
    end
catch
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
