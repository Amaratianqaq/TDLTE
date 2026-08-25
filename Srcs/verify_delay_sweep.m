%% verify_delay_sweep.m - 0x67延迟命令 + 采集时序扫描
% 假设: 若采集窗口与发射突发错位, 宽带OFDM帧整体错过(窄带单音持续存在不受影响)。
% 扫描: A) 0x67命令的延迟字(第6~7字节, uint16小端) 0~65535
%       B) 发完数据后到Get_IQ之间的停顿 0~3秒
% 判定: 某档 rho 显著上升(>0.2) -> 找到对齐时序, 写入OFDM_RFLoopback。
% 前置: 保持上位机当前设置(衰减15/增益5), 天线不动。约3~5分钟。

clear; clc;

srcDir = fileparts(mfilename('fullpath'));
addpath(srcDir);
addpath(fullfile(srcDir, 'Tx'));
addpath(fullfile(srcDir, 'Rx'));

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';

% OFDM发送信号 (峰值2047, 不PAPR削减)
prm = GetLTEParams();
rng(42);
txIQ = OFDM_TxMain(prm);
txIQ = txIQ(:).';

%% A) 0x67延迟字扫描 (第6~7字节)
Dvals = [0 1 2 4 8 16 32 64 128 256 512 1024 2048 4096 8192 12288 ...
         16384 20480 24576 28672 30720 61440 65535];
fprintf('==== A) 0x67延迟字扫描 ====\n');
fprintf('%-8s %-8s %-8s %-10s\n', '延迟字', 'rho', '包络', '帧偏移');
bestA = struct('d',0,'rho',0);
for k = 1:numel(Dvals)
    d = Dvals(k);
    lo = dec2hex(mod(d,256),2); hi = dec2hex(floor(d/256),2);
    cmdDelay = uint8(hex2dec([B, '67', {lo,hi}, repmat({'00'},1,13)]));
    [rho, rhoEnv, delay] = runOne(cmdDelay, 0, txIQ, prm, cmdSync, cmdRouter, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    fprintf('%-8d %-8.4f %-8.3f %-10d\n', d, rho, rhoEnv, delay);
    if rho > bestA.rho, bestA = struct('d',d,'rho',rho); end
end
fprintf('A最佳: 延迟字=%d (rho=%.4f)\n', bestA.d, bestA.rho);

%% B) 停顿扫描 (发完数据后等待pause秒再Get_IQ)
Pvals = [0 0.25 0.5 1 2 3];
fprintf('\n==== B) 采集前停顿扫描 ====\n');
fprintf('%-8s %-8s %-8s %-10s\n', '停顿s', 'rho', '包络', '帧偏移');
bestB = struct('p',0,'rho',0);
for k = 1:numel(Pvals)
    p = Pvals(k);
    cmdDelay0 = uint8(hex2dec([B, '67', repmat({'00'},1,15)]));
    [rho, rhoEnv, delay] = runOne(cmdDelay0, p, txIQ, prm, cmdSync, cmdRouter, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    fprintf('%-8g %-8.4f %-8.3f %-10d\n', p, rho, rhoEnv, delay);
    if rho > bestB.rho, bestB = struct('p',p,'rho',rho); end
end
fprintf('B最佳: 停顿=%gs (rho=%.4f)\n', bestB.p, bestB.rho);

%% C) 组合: 最佳延迟字 + 最佳停顿
if bestA.rho > 0.05 || bestB.rho > 0.05
    lo = dec2hex(mod(bestA.d,256),2); hi = dec2hex(floor(bestA.d/256),2);
    cmdDelayC = uint8(hex2dec([B, '67', {lo,hi}, repmat({'00'},1,13)]));
    [rho, rhoEnv, delay] = runOne(cmdDelayC, bestB.p, txIQ, prm, cmdSync, cmdRouter, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    fprintf('\nC) 组合(延迟=%d, 停顿=%gs): rho=%.4f 包络=%.3f 帧偏移=%d\n', ...
        bestA.d, bestB.p, rho, rhoEnv, delay);
end

fprintf('\n>> 若某档rho>0.2: 时序找到了, 把该延迟字/停顿写入OFDM_RFLoopback;\n');
fprintf('   若全部≈0.1: 时序不是主因, 设备宽带通路受限, 走内部环回/换设备路线。\n');

%% 记录结果到日志
logResult('verify_delay_sweep', struct('bestDelay',bestA.d,'bestDelayRho',bestA.rho, ...
    'bestPause',bestB.p,'bestPauseRho',bestB.rho));

%% ================= 函数 =================
function [rho, rhoEnv, delay] = runOne(cmdDelay, pauseS, txIQ, prm, cmdSync, cmdRouter, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
rxIQ = udpLoopback(cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, pauseS, SAMPLE_LENGTH, pcip, xsrpip);
if isempty(rxIQ), rho = 0; rhoEnv = 0; delay = 0; return; end
% 快速频偏搜索 ±2000Hz/250 + 细搜
r = rxIQ(:).' - mean(rxIQ);
v = txIQ(:).' - mean(txIQ);
N = numel(r); n = 0:N-1; NFFT = prm.NFFT;
best = struct('cfo',0,'rho',0,'lag',0);
g = -2000:250:2000;
for k = 1:numel(g)
    rc = r .* exp(-1j*2*pi*g(k)*n/NFFT);
    [rho, lag] = xp(rc, v);
    if rho > best.rho, best = struct('cfo',g(k),'rho',rho,'lag',lag); end
end
g2 = best.cfo + (-125:62.5:125);
for k = 1:numel(g2)
    rc = r .* exp(-1j*2*pi*g2(k)*n/NFFT);
    [rho, lag] = xp(rc, v);
    if rho > best.rho, best = struct('cfo',g2(k),'rho',rho,'lag',lag); end
end
rho = best.rho; delay = best.lag;
% 平滑包络相关
X = fft(txIQ, N); Y = fft(rxIQ, N);
f = (0:N-1)/N*30.72e6; f(f>15.36e6) = f(f>15.36e6) - 30.72e6;
band = abs(f) < 9.5e6 & abs(f) > 0.2e6;
smX = movmean(abs(X),150); smY = movmean(abs(Y),150);
cc = corrcoef(smX(band), smY(band)); rhoEnv = cc(1,2);
end

function [rho, lag] = xp(a, b)
c = xcorr(a, b);
[cmax, imax] = max(abs(c));
lag = imax - (numel(c)+1)/2;
rho = cmax / (sqrt(sum(abs(a).^2)) * sqrt(sum(abs(b).^2)));
end

function rxIQ = udpLoopback(cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, pauseS, SAMPLE_LENGTH, pcip, xsrpip)
rxIQ = [];
% 发送数据: OFDM (峰值2047, 与OFDM_RFLoopback相同的打包)
prm = GetLTEParams(); rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
dataIQ = zeros(1, SAMPLE_LENGTH*2);
dataIQ(1,1:2:end-1) = real(txIQ); dataIQ(1,2:2:end) = imag(txIQ);
dataIQ = dataIQ .* (2047/max(abs(dataIQ)));
dataIQ = fix(dataIQ);
for n = 1:numel(dataIQ)
    if dataIQ(n) > 2047, dataIQ(n) = 2047;
    elseif dataIQ(n) < 0, dataIQ(n) = 4096 + dataIQ(n); end
end
dataIQ(1,1:2:end-1) = dataIQ(1,1:2:end-1) .* 16;
dataIQ(1,2:2:end) = fix(dataIQ(1,2:2:end)./256) + rem(dataIQ(1,2:2:end),256).*256;
dataIQ = uint16(dataIQ);

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
    if pauseS > 0, pause(pauseS); end
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
catch ME
    fprintf('    UDP错误: %s\n', ME.message);
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
