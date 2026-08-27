%% rx_diag_data.m - 数据相关性诊断: 区分"设备回环我们发送的帧" vs "固定自身内容"
% 背景: rx_accum_v2 显示 对消前rho=0.29 -> 对消后rho=0.08 (内容减掉后信号更弱),
%       说明 rho=0.29 的相关度主要来自"内容本身与TX参考的相关"。
% 本脚本用两组不同数据 X/Y 判定该相关是否随发送数据变化:
%   序列: 发X->采A_X | 不发->采C1 | 发Y->采A_Y | 不发->采C2
% 判读:
%   A) 若 rho(C1,X) 高(~0.3) 而 rho(C1,Y) 低  -> C1 含"上一次发送X的回环"
%      => 设备在回环我们发的帧(内部数字环回), 0.29相关来自回环副本
%      => 下一步: 关/改设备内部环回(0x68命令值), 或确认环回延迟>CP造成的ISI
%   B) 若 rho(A_X,X)≈rho(A_X,Y) (相关不随数据变) -> 固定内容假设
%      => 内容与数据无关但仍与TX参考相关(奇怪), 代码侧到头, 走厂商固件
%   C) 若 rho(A_X,X)>>rho(A_X,Y) -> 直接信号存在且随数据变 -> 信号其实在, 继续调接收
% 使用: 直连+1GHz+衰减2/增益5 状态直接运行, 约40秒, 勿碰设备。

clear; clc; close all;
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

%% 两组不同数据: X(rng42) 和 Y(rng7)
rng(42); txX = OFDM_TxMain(prm); txX = txX(:).';
rng(7);  txY = OFDM_TxMain(prm); txY = txY(:).';
dataX = packIQ(txX, SAMPLE_LENGTH);
dataY = packIQ(txY, SAMPLE_LENGTH);

fprintf('==== 数据相关性诊断 (直连/1GHz, 勿碰设备) ====\n');

%% 序列: A_X, C1, A_Y, C2
A_X = udpLoopback(dataX, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
C1  = udpLoopback([],     cmdSync, cmdRouter, cmdDelay, cmdTx, [],     cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
A_Y = udpLoopback(dataY, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
C2  = udpLoopback([],     cmdSync, cmdRouter, cmdDelay, cmdTx, [],     cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if isempty(A_X)||isempty(C1)||isempty(A_Y)||isempty(C2)
    error('有帧未收到, 检查设备/网络');
end
A_X = A_X(:).'; C1 = C1(:).'; A_Y = A_Y(:).'; C2 = C2(:).';

%% 全部 rho (对X / 对Y)
rho = @(r, v) syncRho(r, v, prm);
fprintf('\n---- rho(回传, TX参考) ----\n');
fprintf('rho(A_X, X)=%.4f | rho(A_X, Y)=%.4f   (A_X是否随数据变)\n', rho(A_X, txX), rho(A_X, txY));
fprintf('rho(A_Y, Y)=%.4f | rho(A_Y, X)=%.4f   (A_Y是否随数据变)\n', rho(A_Y, txY), rho(A_Y, txX));
fprintf('rho(C1,  X)=%.4f | rho(C1,  Y)=%.4f   (C1含上一次发送X的回环?)\n', rho(C1, txX), rho(C1, txY));
fprintf('rho(C2,  X)=%.4f | rho(C2,  Y)=%.4f   (C2含上一次发送Y的回环?)\n', rho(C2, txX), rho(C2, txY));

%% 控制帧 vs 信号帧 共享程度
corrN = @(a,b) max(abs(xcorr(a-mean(a), b-mean(b)))) / ...
    (sqrt(sum(abs(a-mean(a)).^2)) * sqrt(sum(abs(b-mean(b)).^2)));
fprintf('\n---- 帧间共享度 ----\n');
fprintf('corr(A_X, C1)=%.4f | corr(A_Y, C2)=%.4f | corr(C1, C2)=%.4f\n', ...
    corrN(A_X, C1), corrN(A_Y, C2), corrN(C1, C2));

%% 对消实验验证: A_X - alpha*C1 后的 rho (对照rx_accum_v2)
cX = xcorr(A_X - mean(A_X), C1 - mean(C1));
[~, imax] = max(abs(cX));
lagC = imax - (numel(cX)+1)/2;
C1_al = circshift(C1, -lagC);
alpha = sum(conj(C1_al).*A_X) / sum(abs(C1_al).^2);
D = A_X - alpha*C1_al;
fprintf('\n---- 对消验证 (A_X - alpha*C1) ----\n');
fprintf('alpha=%.3f%+.3fj | 消掉%.1f%%能量 | rho(D,X)=%.4f (对消前rho(A_X,X)=%.4f)\n', ...
    real(alpha), imag(alpha), (1-sum(abs(D).^2)/sum(abs(A_X).^2))*100, rho(D, txX), rho(A_X, txX));

%% 判定
fprintf('\n==== 判定 ====\n');
rC1X = rho(C1, txX); rC1Y = rho(C1, txY);
rAXx = rho(A_X, txX); rAXy = rho(A_X, txY);
if rC1X > 0.15 && rC1Y < rC1X*0.5
    fprintf('>> A) 回环假设成立: C1(没发数据)里含有上一次发送X的副本(rho=%.3f), 发Y后相关消失(%.3f)。\n', rC1X, rC1Y);
    fprintf('   设备在"内部回环"我们发送的帧 -> 0.29的相关来自回环副本, 不是天线直达信号。\n');
    fprintf('   下一步: 改/关 0x68 路由命令的内部环回, 或确认回环延迟>CP造成的ISI。\n');
elseif abs(rAXx - rAXy) < 0.05
    fprintf('>> B) 固定内容假设: rho(A_X,X)=%.3f ≈ rho(A_X,Y)=%.3f, 相关不随数据变。\n', rAXx, rAXy);
    fprintf('   内容是固定模式且与TX参考巧合相关 -> 代码侧已到头, 需厂商TDLTE环境/固件。\n');
else
    fprintf('>> C) 直接信号存在: rho(A_X,X)=%.3f >> rho(A_X,Y)=%.3f, 相关随数据变 -> 信号在回传里!\n', rAXx, rAXy);
    fprintf('   继续调接收链(回环延迟>CP则先解决ISI)。\n');
end

%% 记录
logResult('rx_diag_data', struct('rhoAX_X',rAXx,'rhoAX_Y',rAXy,'rhoAY_Y',rho(A_Y,txY), ...
    'rhoAY_X',rho(A_Y,txX),'rhoC1_X',rC1X,'rhoC1_Y',rC1Y,'rhoC2_X',rho(C2,txX), ...
    'rhoC2_Y',rho(C2,txY),'alpha',alpha,'rhoD',rho(D,txX)));
save('rx_diag_data_result.mat', 'A_X','C1','A_Y','C2','txX','txY','alpha','D');

%% ================= 局部函数 =================
function r = syncRho(rx, tx, prm)
[~, r, ~, ~] = syncLTE(rx, tx, prm);
end

function dataIQ = packIQ(txIQ, SAMPLE_LENGTH)
txIQ = txIQ(:).';
len = SAMPLE_LENGTH * 2;
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = real(txIQ);
dataIQ(1,2:2:len)   = imag(txIQ);
dataIQ = dataIQ .* (2047 / max(abs(dataIQ)));
dataIQ = fix(dataIQ);
for n = 1:len
    if dataIQ(n) > 2047
        dataIQ(n) = 2047;
    elseif dataIQ(n) < 0
        dataIQ(n) = 4096 + dataIQ(n);
    end
end
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1) .* 16;
dataIQ(1,2:2:len)   = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
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
try
    fclose(udp_obj);
catch
end
delete(udp_obj); clear udp_obj;
end
