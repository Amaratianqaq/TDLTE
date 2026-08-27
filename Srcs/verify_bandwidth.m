%% verify_bandwidth.m - 真实环境带宽响应测绘 (定位故障模块)
% 目的: 判定"是带宽受限(硬件特性)"还是"模块坏了(数据都没通过)"。
% 方法: 发送以5MHz为中心、不同带宽(BW=0.1/1/3/6/9MHz)的随机数据信号,
%       每次用syncLTE测回传保真度rho。
% 判读:
%   窄带rho高、宽带rho低  -> 链路带宽受限(天线耦合/接收机滤波器) -> 硬件特性
%   所有带宽rho都低        -> 数据未通过(发射数据路径/接收采集/格式) -> 查模块
%   所有带宽rho都高        -> 链路正常! 问题在我们的OFDM具体参数
% 前置: 保持当前环境(AC耦合/正常模式/TX1-RX1/衰减2/增益5), 勿碰天线。

clear; clc;
addpath(fileparts(mfilename('fullpath')));

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
prm = GetLTEParams();

A = 1000;                    % 发送峰值(12位)
BWs = [0 0.1 1 3 6 9] * 1e6; % 0=纯单音(恒定包络), 其余=5MHz中心各带宽随机数据
fprintf('==== 真实环境带宽响应测绘 (5MHz中心, 勿碰天线/设备) ====\n');
fprintf('%-8s %-10s %-10s %-10s\n', 'BW(MHz)', 'rho', '频偏Hz', '帧偏移');

results = [];
for k = 1:numel(BWs)
    bw = BWs(k);
    if bw == 0
        % 纯5MHz单音 (恒定包络)
        x = A * exp(1j*2*pi*5e6*(0:SAMPLE_LENGTH-1)/Fs);
    else
        % 以5MHz为中心、±bw/2带宽的随机复信号
        rng(k);
        bb = randn(1, SAMPLE_LENGTH) + 1j*randn(1, SAMPLE_LENGTH);
        X = fft(bb);
        f = (0:SAMPLE_LENGTH-1)/SAMPLE_LENGTH*Fs;  f(f>Fs/2) = f(f>Fs/2) - Fs;
        X(abs(f) > bw/2) = 0;
        x = ifft(X) .* exp(1j*2*pi*5e6*(0:SAMPLE_LENGTH-1)/Fs);
        x = x / max(abs(x)) * A;                 % 有符号, 峰值A
    end

    dataIQ = packComplex(x);
    rxIQ = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(rxIQ)
        fprintf('%-8.1f %-10s\n', bw/1e6, '未收到');
        continue;
    end
    [~, rho, cfoHz, delay] = syncLTE(rxIQ, x, prm);
    fprintf('%-8.1f %-10.4f %-10.1f %-10d\n', bw/1e6, rho, cfoHz, delay);
    results = [results; struct('bw',bw,'rho',rho,'cfo',cfoHz,'delay',delay)]; %#ok<AGROW>
    save(sprintf('bw_%dMHz.mat', bw/1e6), 'rxIQ', 'x', 'rho');
end

%% 判定
fprintf('\n==== 判定 ====\n');
if ~isempty(results)
    rhoTone = results(1).rho;                   % BW=0 纯单音
    rhoNarrow = results(2).rho;                 % 0.1MHz
    rhoWide   = results(end).rho;               % 9MHz
    fprintf('纯单音rho=%.3f | 窄带(0.1MHz)rho=%.3f | 宽带(9MHz)rho=%.3f\n', ...
        rhoTone, rhoNarrow, rhoWide);
    if rhoTone > 0.5 && rhoNarrow < 0.2
        fprintf('>> 单音能相干通过, 但任何带调制数据都不通 -> 接收路径是限幅/包络检测型!\n');
        fprintf('   设备只保留载波(频率/幅度), 丢弃调制波形 -> 宽带OFDM不可能通过 -> 硬件特性。\n');
    elseif rhoTone < 0.2
        fprintf('>> 连纯单音相干rho也仅%.3f -> 回传与发送波形无相干, 发射数据路径/接收采集有问题。\n', rhoTone);
    else
        fprintf('>> 单音rho=%.3f, 数据rho=%.3f -> 调制有一定通过但弱, 继续查干扰/参数。\n', rhoTone, rhoNarrow);
    end
end
logResult('verify_bandwidth', struct('bwsMHz',BWs/1e6,'rhos',[results.rho]));

%% ================= 函数 =================
function dataIQ = packComplex(x)
% 复信号 -> 12位I/Q打包 (与OFDM_RFLoopback一致)
x = x(:).';
SAMPLE_LENGTH = numel(x);
len = SAMPLE_LENGTH * 2;
txI = real(x); txQ = imag(x);
txI(txI<0) = txI(txI<0) + 4096;
txQ(txQ<0) = txQ(txQ<0) + 4096;
txI = fix(txI); txQ = fix(txQ);
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = txI;  dataIQ(1,2:2:len) = txQ;
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1).*16;
dataIQ(1,2:2:len) = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
dataIQ = uint16(dataIQ);
end

function rxIQ = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
rxIQ = [];
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
        rxIQ = iv + 1j*qv;
    end
catch
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
