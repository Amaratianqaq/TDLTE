%% verify_interf_cancel.m - 干扰对消实验 (Rx端最后一种信号提取算法)
% 背景: 回传=确定性干扰(约60%)+随机噪声+弱信号(约5%)。平均只能消随机部分。
% 方法: 采一帧"控制帧"(不发数据, 只有干扰) C, 对齐后从"信号帧" A 中减去,
%       消除确定性干扰, 提取弱信号。
% 步骤: 1) 发OFDM采集A  2) 不发数据采集C  3) 对齐C到A(互相关找延迟)
%       4) 最小二乘求增益α, A' = A - α*C  5) 测rho与CRC对比
% 前置: 保持当前环境(AC耦合/正常模式/衰减2/增益5), 勿碰天线。

clear; clc;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';
prm = GetLTEParams();

%% 1) 发送OFDM采集信号帧A
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
rxA = udpLoopback(txIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if isempty(rxA), error('信号帧未收到'); end

%% 2) 控制帧C: 不发送数据
rxC = udpLoopback([], cmdSync, cmdRouter, cmdDelay, cmdTx, [], cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if isempty(rxC), error('控制帧未收到'); end

%% 3) 对齐C到A (两者共享确定性干扰, 互相关峰给出相对延迟)
N = numel(rxA);
A = rxA(:).';  C = rxC(:).';
c = xcorr(A, C);
[~, imax] = max(abs(c));
lag = imax - (numel(c)+1)/2;
C_al = circshift(C, -lag);          % 对齐
fprintf('控制帧相对延迟=%d样点, 相关=%.4f\n', lag, max(abs(c))/sqrt(sum(abs(A).^2)*sum(abs(C).^2)));

%% 4) 最小二乘增益与对消
alpha = sum(conj(C_al) .* A) / sum(abs(C_al).^2);
A2 = A - alpha * C_al;
fprintf('对消增益alpha=%.3f+%.3fj (对消掉 %.1f%% 能量)\n', ...
    real(alpha), imag(alpha), (1 - sum(abs(A2).^2)/sum(abs(A).^2))*100);

%% 5) 对比: 对消前后 rho 与 CRC
[~, rhoA]  = syncLTE(A,  txIQ, prm);
[~, rhoA2] = syncLTE(A2, txIQ, prm);
fprintf('\n对消前 rho=%.4f | 对消后 rho=%.4f\n', rhoA, rhoA2);

try
    [~, crcA]  = OFDM_RxMain(A,  prm);
    [~, crcA2] = OFDM_RxMain(A2, prm);
    fprintf('对消前 CRC=%d | 对消后 CRC=%d\n', crcA, crcA2);
catch ME
    fprintf('CRC出错: %s\n', ME.message);
end

%% 6) 多帧对消增强: 采8帧信号平均再减去8帧控制平均
fprintf('\n---- 多帧对消增强 (K=8) ----\n');
K = 8;
sA = zeros(1, N); sC = zeros(1, N);
for k = 1:K
    ra = udpLoopback(txIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    rc = udpLoopback([], cmdSync, cmdRouter, cmdDelay, cmdTx, [], cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(ra) || isempty(rc), fprintf('第%d次未收到\n', k); continue; end
    [sa, ~, ~, ~, pha] = syncLTE(ra(:).', txIQ, prm);
    sA = sA + sa .* exp(-1j*pha);           % 信号帧相位对齐累加
    % 控制帧: 用共享干扰结构对齐到信号帧的时序
    cc = xcorr(ra(:).', rc(:).');
    [~, ii] = max(abs(cc));
    lc = ii - (numel(cc)+1)/2;
    sC = sC + circshift(rc(:).', -lc);
end
Aavg = sA / K;  Cavg = sC / K;
alpha2 = sum(conj(Cavg) .* Aavg) / sum(abs(Cavg).^2);
Aavg2 = Aavg - alpha2 * Cavg;
[~, rhoM1] = syncLTE(Aavg,  txIQ, prm);
[~, rhoM2] = syncLTE(Aavg2, txIQ, prm);
fprintf('平均后 rho=%.4f | 平均+对消 rho=%.4f\n', rhoM1, rhoM2);
try
    [~, crcM1] = OFDM_RxMain(Aavg,  prm);
    [~, crcM2] = OFDM_RxMain(Aavg2, prm);
    fprintf('平均后 CRC=%d | 平均+对消 CRC=%d\n', crcM1, crcM2);
catch ME
    fprintf('CRC出错: %s\n', ME.message);
end
save('interf_cancel_result.mat', 'A', 'C', 'A2', 'Aavg', 'Aavg2', 'rhoA', 'rhoA2', 'rhoM1', 'rhoM2');
logResult('verify_interf_cancel', struct('rho_before',rhoA,'rho_after',rhoA2, ...
    'rhoAvg_before',rhoM1,'rhoAvg_after',rhoM2));

%% ================= 函数 =================
function rxIQ = udpLoopback(txIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip)
rxIQ = [];
if isempty(txIQ)
    % 控制帧: 无数据
    dataIQ = [];
else
    txIQ = txIQ(:).';
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
end

udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
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
        rxIQ = iv + 1j*qv;
    end
catch
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
