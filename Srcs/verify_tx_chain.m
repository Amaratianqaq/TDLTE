%% verify_tx_chain.m - Tx端发射链路完整验证
% 目标: 判定XSRP的发射端(TX)是否真的在辐射我们UDP送进去的信号。
% 两个决定性判据(纯代码, 无需外部设备):
%   A) 频率跟随: 发送 3/5/7/9MHz 单音, 回传峰必须精确出现在同一频率
%      (若峰是设备自产, 不会跟随我们改的频率)
%   B) 幅度跟随: 5MHz单音幅度 600->1200 (6dB), 回传峰应上升约6dB
%      (若峰是设备自产, 幅度不会随我们发送的电平变化)
% 判定:
%   频率跟随>=3/4 且 幅度跟随成立 -> Tx端确认在发射, 信号走了天线辐射路径
%   频率跟随失败                   -> 回传与发送无关, 发射链路未工作
% 前置: 天线接好(TX1/RX1), 上位机TX=RX频点一致, 衰减2dB左右, 真实系统。
% 使用: 直接运行, 约30秒(7次UDP收发)。勿碰天线。

clear; clc;
addpath(fileparts(mfilename('fullpath')));   % 确保 logResult 可用

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

fprintf('==== Tx端发射链路验证 (勿碰天线/设备) ====\n');

%% A) 频率跟随测试: 3/5/7/9MHz
F0S = [3 5 7 9] * 1e6;
A0  = 1200;
ratios = zeros(1, numel(F0S));
for k = 1:numel(F0S)
    f0 = F0S(k);
    dataIQ = makeTone(f0, A0, SAMPLE_LENGTH, Fs);
    z = toneCapture(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(z)
        fprintf('频率 %dMHz : 未收到数据\n', f0/1e6);
        ratios(k) = 0;
        continue;
    end
    [own, peakFreq] = toneRatio(z, f0, Fs);
    ratios(k) = own;
    fprintf('频率 %dMHz : 回传峰在 %.3fMHz (比值 %.0fx) -> %s\n', ...
        f0/1e6, peakFreq/1e6, own, ternary(own > 20, '跟随', '无'));
end
freqFollow = sum(ratios > 20);

%% B) 幅度跟随测试: 5MHz 幅度 600 vs 1200
f0 = 5e6;
[~, p1] = ampTest(f0, 600,  cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip, Fs);
[~, p2] = ampTest(f0, 1200, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip, Fs);
ampDelta = 10*log10(p2 / max(p1, eps));
fprintf('幅度跟随: 600->1200 回传峰变化 %.1f dB (理论+6dB) -> %s\n', ...
    ampDelta, ternary(abs(ampDelta - 6) < 4, '跟随', '不跟随'));
ampFollow = abs(ampDelta - 6) < 4;

%% 判定
fprintf('\n==== 判定 ====\n');
if freqFollow >= 3 && ampFollow
    fprintf('>> Tx端确认在发射!! (频率跟随 %d/4 + 幅度跟随%.1fdB)\n', freqFollow, ampDelta);
    fprintf('   信号确实从TX天线辐射并经天线耦合被接收。问题只在宽带OFDM的通过性。\n');
elseif freqFollow >= 3
    fprintf('>> 频率跟随 %d/4 (幅度跟随%.1fdB未达标) -> Tx发射基本确认, 幅度响应待查。\n', freqFollow, ampDelta);
else
    fprintf('>> 频率跟随仅 %d/4 -> 回传与发送频率无关, 发射链路未工作。\n', freqFollow);
    fprintf('   检查: 上位机发射通道/频点、天线、设备初始化、衰减。\n');
end

%% 记录结果到日志
logResult('verify_tx_chain', struct('freqFollow',freqFollow,'ratios',ratios, ...
    'ampDelta_dB',ampDelta,'ampFollow',ampFollow));

%% ================= 函数 =================
function [ratio, peakFreq] = toneRatio(z, f0, Fs)
N = numel(z);
X = fft(z(:).', N);
f = (0:N-1)/N*Fs; f(f>Fs/2) = f(f>Fs/2) - Fs;
P = abs(X).^2;
mask = abs(f) > 0.2e6;
peakBand = abs(abs(f) - f0) < 0.3e6;
localBand = abs(abs(f) - f0) < 2e6 & ~peakBand;
localMed = median(P(localBand));
[pmax, pi_] = max(P(peakBand));
ratio = pmax / max(localMed, eps);
ff = f(peakBand);
peakFreq = ff(pi_);
end

function [ratio, p] = ampTest(f0, A, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip, Fs)
dataIQ = makeTone(f0, A, SAMPLE_LENGTH, Fs);
z = toneCapture(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if isempty(z), ratio = 0; p = eps; return; end
N = numel(z);
X = fft(z(:).', N);
f = (0:N-1)/N*Fs; f(f>Fs/2) = f(f>Fs/2) - Fs;
P = abs(X).^2;
peakBand = abs(abs(f) - f0) < 0.3e6;
mask = abs(f) > 0.2e6;
localBand = abs(abs(f) - f0) < 2e6 & ~peakBand;
localMed = median(P(localBand));
p = max(P(peakBand));
ratio = p / max(localMed, eps);
end

function dataIQ = makeTone(f0, A, SAMPLE_LENGTH, Fs)
n = 0:SAMPLE_LENGTH-1;
txI = A*cos(2*pi*f0*n/Fs); txQ = A*sin(2*pi*f0*n/Fs);
txI(txI<0) = txI(txI<0)+4096; txQ(txQ<0) = txQ(txQ<0)+4096;
txI = fix(txI); txQ = fix(txQ);
len = SAMPLE_LENGTH*2;
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = txI;  dataIQ(1,2:2:len) = txQ;
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1).*16;
dataIQ(1,2:2:len) = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
dataIQ = uint16(dataIQ);
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

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end
