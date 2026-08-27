%% tx_tone_test.m - 发射通路自检 v3 (控制组 + 多频点单音)
% 目的: 判明回传里出现的5MHz峰到底是不是我们发出去的。
%  A) 控制组: 不发送任何IQ数据, 直接Get_IQ抓回看频谱
%     -> 若控制组也有5MHz峰 => 5MHz是设备自产信号, 之前"突破"是假象
%  B) 多频组: 同时发送 3/5/8 MHz 三个单音, 抓回看频谱
%     -> 峰跟随发送频率(3/5/8MHz处出现) => 环回真的通! 问题只在OFDM链路处理
%     -> 只出现5MHz峰(其他没有) => 5MHz自产, 环回仍不通
% 前置: 上位机 TX_Freq=RX_Freq, 衰减2dB, 真实系统, 天线连接 (保持与最近一次相同状态!)
% 使用: 直接运行, 约30秒(两次UDP收发)

clear; clc;
addpath(fileparts(mfilename('fullpath')));   % 确保 logResult 可用

SAMPLE_LENGTH = 30720;
Fs = 30.72e6;
F0S = [3e6 5e6 8e6];      % 多频点单音
Amp = 600;                % 每音幅度(3音叠加峰值<=1800 < 2047, 不削波)

B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));

pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';

%% A) 控制组: 不发送数据
fprintf('==== A) 控制组: 不发送任何IQ数据 ====\n');
[zC, okC] = toneCapture([], cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if okC
    reportSpectrum(zC, Fs, '控制组(未发送)');
else
    fprintf('控制组未收到数据\n');
end

%% B) 多频组: 3/5/8 MHz 三个单音
fprintf('\n==== B) 多频组: 发送 %.0f/%.0f/%.0f MHz 三个单音 ====\n', F0S/1e6);
n = 0:SAMPLE_LENGTH-1;
txI = zeros(1,SAMPLE_LENGTH); txQ = zeros(1,SAMPLE_LENGTH);
for k = 1:numel(F0S)
    txI = txI + Amp*cos(2*pi*F0S(k)*n/Fs);
    txQ = txQ + Amp*sin(2*pi*F0S(k)*n/Fs);
end
txI(txI<0) = txI(txI<0) + 4096;
txQ(txQ<0) = txQ(txQ<0) + 4096;
txI = fix(txI); txQ = fix(txQ);
len = SAMPLE_LENGTH*2;
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = txI;
dataIQ(1,2:2:len)   = txQ;
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1) .* 16;
dataIQ(1,2:2:len)   = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
dataIQ = uint16(dataIQ);

[zT, okT] = toneCapture(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if okT
    reportSpectrum(zT, Fs, '多频组(3/5/8MHz)');
end

%% C) 结论
fprintf('\n==== 判定 ====\n');
if okC && okT
    peaksC = topPeaks(zC, Fs);
    peaksT = topPeaks(zT, Fs);
    has3 = any(abs(peaksT - 3e6) < 0.3e6);
    has5 = any(abs(peaksT - 5e6) < 0.3e6);
    has8 = any(abs(peaksT - 8e6) < 0.3e6);
    c5 = any(abs(peaksC - 5e6) < 0.3e6);
    fprintf('控制组5MHz峰: %d ; 多频组 3MHz:%d 5MHz:%d 8MHz:%d\n', c5, has3, has5, has8);
    if ~c5 && (has3 || has5 || has8)
        fprintf('>> 峰跟随发送频率, 控制组无峰 -> 射频环回真的通了!!\n');
        fprintf('>> 问题只在OFDM链路处理(定时/频偏/干扰), 继续调 verify_hw_link 的同步即可。\n');
    elseif c5 && ~has3 && ~has8
        fprintf('>> 5MHz峰在控制组就存在且不跟随频率 -> 5MHz是设备自产信号, 环回仍不通。\n');
        fprintf('>> 需要继续查硬件/上位机(与之前结论一致)。\n');
    elseif c5
        fprintf('>> 控制组也有5MHz峰, 但其他频率也回来了 -> 混合情况, 设备自产5MHz与环回共存。\n');
    else
        fprintf('>> 多频组峰未跟随频率 -> 环回仍不通。\n');
    end
end

%% 记录结果到日志
if exist('c5','var')
    logResult('tx_tone_test', struct('ctrl5MHz',c5,'multi3',has3,'multi5',has5,'multi8',has8));
end

%% ================= 局部函数 =================
function [z, ok] = toneCapture(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
ok = false; z = [];
try
    fopen(udp_obj);
    fwrite(udp_obj, cmdSync,   'uint8');
    fwrite(udp_obj, cmdRouter, 'uint8');
    fwrite(udp_obj, cmdDelay,  'uint8');
    fwrite(udp_obj, cmdTx,     'uint8');
    if ~isempty(dataIQ)
        fwrite(udp_obj, cmdSend, 'uint8');
        SEND_PACKET_LENGTH = 512;
        for pn = 1:fix(SAMPLE_LENGTH*2/SEND_PACKET_LENGTH)
            fwrite(udp_obj, dataIQ(1,((pn-1)*SEND_PACKET_LENGTH+1):(pn*SEND_PACKET_LENGTH)), 'uint16');
        end
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
        ok = true;
    end
catch ME
    fprintf('    UDP错误: %s\n', ME.message);
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end

function reportSpectrum(z, Fs, tag)
N = numel(z);
z = z(:).';
X = fft(z, N);
f = (0:N-1)/N*Fs;  f(f>Fs/2) = f(f>Fs/2) - Fs;
P = abs(X).^2;
mask = abs(f) > 0.2e6;
med = median(P(mask));
dcR = max(P(abs(f) < 0.05e6)) / med;
fprintf('%s: DC/噪声=%.1fx, 0.2~1MHz带峰=%.3g (%.1fx)\n', tag, dcR, ...
    max(P(abs(f)<1e6 & mask)), max(P(abs(f)<1e6 & mask))/med);
[Ps, idxs] = sort(P(mask), 'descend');
fsP = f(mask); fsP = fsP(idxs);
fprintf('%s Top 8 峰值(除直流):\n', tag);
for k = 1:min(8, numel(Ps))
    fprintf('  %8.3f MHz : %9.3g (%.1f dB)\n', fsP(k)/1e6, Ps(k), 10*log10(Ps(k)+eps));
end
figure('Name',tag);
plot(f/1e6, 10*log10(P+eps)); grid on;
xlabel('MHz'); ylabel('dB'); title(tag);
xlim([-15.36 15.36]);
end

function peaks = topPeaks(z, Fs)
N = numel(z);
X = fft(z(:).', N);
f = (0:N-1)/N*Fs;  f(f>Fs/2) = f(f>Fs/2) - Fs;
P = abs(X).^2;
mask = abs(f) > 0.2e6;
[Ps, idxs] = sort(P(mask), 'descend');
fsP = f(mask); fsP = fsP(idxs);
% 取超过噪声中位数100x的峰
med = median(P(mask));
peaks = fsP(Ps > 100*med);
end
