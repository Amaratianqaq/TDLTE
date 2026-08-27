%% rx_accum_v2.m - 交替差分对消 + 多帧相干平均 (解决设备自身内容掩蔽)
% 背景(实测数据结论):
%   回传 = 确定性设备内容(约89%能量, 同数据两次回传相关0.81, 与发不发数据几乎无关)
%        + 我们的OFDM信号(约11%能量, rho^2≈0.11)
%        + 随机噪声
%   普通K帧平均只对随机噪声有效(实测K=16仅+0.55dB), 因为设备内容是确定性的。
% 本脚本方法:
%   1) 交替采集: A帧(发数据) / C帧(不发数据), 用同一命令序列(仅跳过数据发送)
%   2) C与A共享同一确定性内容, 用互相关把C对齐到A, 最小二乘求增益alpha
%   3) D = A - alpha*C  -> 确定性内容被消掉, 只剩 信号 + (随机残差)
%   4) 对D做syncLTE(此时内容已消, 同步能锁到我们的信号), 相位对齐后累加K对
%   5) 平均后跑完整收端链看CRC; 并输出每子载波SNR(决定性指标)
% 期望: 若内容可消掉9成, 等效SNR提升 ~9dB, 越过~4.5dB门槛 -> CRC=1
% 前置: 直连(射频线+衰减器)或天线均可, 直连内容最稳定效果最好。
%       上位机: 发射频率=接收频率(当前1GHz), 衰减2, 增益5, 真实系统, AC耦合。
%       运行期间勿碰设备/线缆。
% 使用: 直接运行, 输入交替对数量K(默认8, 每对=1帧信号+1帧控制, 约10~15秒/对)。

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

Kstr = input('交替对数量K(默认8, 建议8~16): ', 's');
K = str2double(Kstr); if isnan(K) || K < 1, K = 8; end

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
rng(42);
txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
dataIQ = packIQ(txIQ, SAMPLE_LENGTH);          % 与 OFDM_RFLoopback 一致的打包

fprintf('==== 交替差分对消 K=%d (直连/天线均可, 勿碰设备/线缆) ====\n', K);

%% 0) 内容稳定性预检: 连采两个控制帧, 看确定性内容能复现到什么程度
fprintf('\n[预检] 采集两个控制帧(不发数据)...\n');
c1 = udpLoopback([], cmdSync, cmdRouter, cmdDelay, cmdTx, [], cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
c2 = udpLoopback([], cmdSync, cmdRouter, cmdDelay, cmdTx, [], cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
if isempty(c1) || isempty(c2)
    error('预检未收到控制帧, 检查设备/网络');
end
cc = xcorr(c1(:).'-mean(c1), c2(:).'-mean(c2));
corrC1C2 = max(abs(cc)) / (sqrt(sum(abs(c1-mean(c1)).^2)) * sqrt(sum(abs(c2-mean(c2)).^2)));
fprintf('corr(C1,C2)=%.4f  (>=0.85: 对消有戏; <0.6: 内容不稳定, 对消收益有限)\n', corrC1C2);

%% 主循环
sumD = zeros(1, SAMPLE_LENGTH);
logA = struct('rhoA',{},'rhoD',{},'alpha',{},'removedPct',{},'corrCC',{},'lagC',{});
cPrev = [];                                   % 上一控制帧 (稳定性监测用)
for k = 1:K
    % 1) 信号帧A(发数据) + 控制帧C(不发数据)
    rxA = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    rxC = udpLoopback([], cmdSync, cmdRouter, cmdDelay, cmdTx, [], cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(rxA) || isempty(rxC)
        fprintf('第%d对未收到, 跳过\n', k); continue;
    end
    rxA = rxA(:).'; rxC = rxC(:).';

    % 2) 对消前: 信号帧对TX参考的rho (基线)
    [~, rhoA, ~, ~, ~] = syncLTE(rxA, txIQ, prm);

    % 3) 把C对齐到A (两者共享确定性内容, 内容互相关峰给出相对延迟)
    cX = xcorr(rxA - mean(rxA), rxC - mean(rxC));
    [~, imax] = max(abs(cX));
    lagC = imax - (numel(cX)+1)/2;
    C_al = circshift(rxC, -lagC);

    % 4) 最小二乘增益 + 相减 (原始域直接减, 内容连同其频偏一起被消)
    alpha = sum(conj(C_al) .* rxA) / sum(abs(C_al).^2);
    D = rxA - alpha * C_al;
    removedPct = (1 - sum(abs(D).^2)/sum(abs(rxA).^2)) * 100;

    % 5) 对消后: 内容已消, syncLTE现在锁的是我们的信号
    [Dsync, rhoD, ~, ~, phD] = syncLTE(D, txIQ, prm);
    sumD = sumD + Dsync .* exp(-1j*phD);       % 相位对齐累加

    % 6) 内容稳定性监测 (相邻控制帧)
    if ~isempty(cPrev)
        cX2 = xcorr(rxC - mean(rxC), cPrev - mean(cPrev));
        corrCCk = max(abs(cX2)) / (sqrt(sum(abs(rxC-mean(rxC)).^2)) * sqrt(sum(abs(cPrev-mean(cPrev)).^2)));
    else
        corrCCk = NaN;
    end
    logA(k) = struct('rhoA',rhoA,'rhoD',rhoD,'alpha',alpha, ...
        'removedPct',removedPct,'corrCC',corrCCk,'lagC',lagC); %#ok<AGROW>
    cPrev = rxC;

    fprintf('第%2d对: 对消前rho=%.4f -> 对消后rho=%.4f | alpha=%.3f%+.3fj | 消掉%.1f%%能量 | C对齐滞后%+d | corr(C_k,C_{k-1})=%.4f\n', ...
        k, rhoA, rhoD, real(alpha), imag(alpha), removedPct, lagC, corrCCk);
end

%% 平均后结果
Davg = sumD / K;
[~, rhoAvg] = syncLTE(Davg, txIQ, prm);
fprintf('\n==== 对消+K平均结果 ====\n');
fprintf('对消后平均rho=%.4f (对消前最好单帧约0.33, 仿真CRC=1需要约0.78)\n', rhoAvg);

% 每子载波SNR (决定性指标, 与evm_check同法)
snrPer = subcarrierSNR(Davg, txIQ, prm);
fprintf('每子载波SNR: 平均=%.2f dB | 中位=%.2f dB | 90分位=%.2f dB | >0dB占比=%.1f%%\n', ...
    mean(snrPer), median(snrPer), quantile(snrPer,0.9), 100*mean(snrPer>0));
fprintf('参考: 离线CRC=1门槛约 +3.3~5dB/子载波\n');

% 完整收端链
try
    [bits, crcOK] = OFDM_RxMain(Davg, prm);
    fprintf('对消+平均后 CRC=%d\n', crcOK);
    if crcOK
        fprintf('>>>>> 链路打通! <<<<<\n');
    end
catch ME
    fprintf('CRC计算出错: %s\n', ME.message);
    crcOK = NaN;
end

save('rx_accum_v2_result.mat', 'Davg', 'rhoAvg', 'snrPer', 'crcOK', 'logA', 'corrC1C2');
fprintf('结果已存 rx_accum_v2_result.mat\n');
logResult('rx_accum_v2', struct('K',K,'corrC1C2',corrC1C2,'rhoAvg',rhoAvg, ...
    'snrAvg',mean(snrPer),'crc',crcOK,'removedPctMean',mean([logA.removedPct])));

%% ================= 局部函数 =================
function dataIQ = packIQ(txIQ, SAMPLE_LENGTH)
% 与 OFDM_RFLoopback.m 完全一致的打包: 峰值2047, 12位补码, I<<4, Q字节交换
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
% 一次UDP收发: 发数据(dataIQ非空)或纯控制(空), 回传解包为复数IQ
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
