function rx_accum_v3()
% rx_accum_v3 - 好状态 大K 帧相干平均 (配置: TX掩码0001 + 采集时隙00)
% 依据: rx_sweep_delay_slot 发现 tx=0001/slot=00 下帧间内容独立性好,
%       平均效率高 (K=8: rho 0.28->0.43, SNR 1.14dB; 默认配置 K=16 仅 0.25dB)。
% 本脚本: 纯相干平均(不做差分), K 可调(默认64), 输出 rho/每子载波SNR/CRC。
% 预期: K=32~64 时每子载波SNR达 3.9~5.2dB, 可能跨过 ~4.5dB 解码门槛。
% 前置: 直连+1GHz+衰减2/增益5+AC耦合+真实系统, 勿碰设备。
% 使用: 直接运行, 输入K (K=64约4分钟, K=128约8分钟)。

addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

Kstr = input('平均帧数K(默认64, 推荐32~128): ', 's');
K = str2double(Kstr); if isnan(K) || K < 1, K = 64; end

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec([B, '68','00','00','00','00','06','00','00', repmat({'00'},1,8)]));
cmdDelay  = mkDelay(0);
cmdTx     = mkTx('0001');          % 只发时隙0
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = mkGet('00','01');      % 采集时隙00
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';

prm = GetLTEParams();
rng(42); txIQ = OFDM_TxMain(prm); txIQ = txIQ(:).';
dataIQ = packIQ(txIQ, SAMPLE_LENGTH);

fprintf('==== K=%d 相干平均 (tx=0001/slot=00, 勿碰设备/线缆) ====\n', K);
sumAcc = zeros(1, SAMPLE_LENGTH);
rhos = zeros(1, K);
for k = 1:K
    rxA = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip);
    if isempty(rxA)
        fprintf('第%d帧未收到, 跳过\n', k); rhos(k) = NaN; continue;
    end
    [rs, rho, ~, ~, ph] = syncLTE(rxA(:).', txIQ, prm);
    sumAcc = sumAcc + rs .* exp(-1j*ph);
    rhos(k) = rho;
    if mod(k, 8) == 0 || k == 1
        fprintf('第%4d帧 rho=%.4f | 已收%d帧均值=%.4f\n', k, rho, sum(~isnan(rhos)), mean(rhos(~isnan(rhos))));
    end
end

nOK = sum(~isnan(rhos));
Davg = sumAcc / max(nOK, 1);
[~, rhoAvg] = syncLTE(Davg, txIQ, prm);
fprintf('\n==== 结果 ====\n');
fprintf('有效帧=%d | 单帧rho均值=%.4f | 平均后rho=%.4f\n', nOK, mean(rhos(~isnan(rhos))), rhoAvg);
snrPer = subcarrierSNR(Davg, txIQ, prm);
fprintf('每子载波SNR: 平均=%.2f dB | 中位=%.2f dB | 90分位=%.2f dB | >0dB占比=%.1f%%\n', ...
    mean(snrPer), median(snrPer), quantile(snrPer, 0.9), 100*mean(snrPer > 0));
fprintf('参考: 解码门槛约 4.2~5.0 dB (本度量口径)\n');
if mean(snrPer) > 4.5
    fprintf('>> 已过门槛! 理论上应能解出。\n');
elseif mean(snrPer) > 2.5
    fprintf('>> 距门槛差 %.1f dB, 加大K到128/256再试。\n', 4.5 - mean(snrPer));
else
    fprintf('>> 距门槛还差 %.1f dB。\n', 4.5 - mean(snrPer));
end

try
    [~, crcOK] = OFDM_RxMain(Davg, prm);
    fprintf('CRC=%d\n', crcOK);
    if crcOK, fprintf('>>>>> 链路打通! <<<<<\n'); end
catch ME
    fprintf('CRC出错: %s\n', ME.message); crcOK = NaN;
end

save('rx_accum_v3_result.mat', 'Davg', 'rhoAvg', 'snrPer', 'crcOK', 'rhos');
try
    logResult('rx_accum_v3', struct('K',K,'rhoAvg',rhoAvg,'snrAvg',mean(snrPer),'crc',crcOK));
catch
end
fprintf('结果已存 rx_accum_v3_result.mat\n');

end

%% ================= 局部函数 =================
function c = mkDelay(d)
c = uint8(hex2dec([{'00','00','99','bb','67'}, {dec2hex(floor(d/256),2), dec2hex(mod(d,256),2)}, repmat({'00'},1,13)]));
end

function c = mkTx(maskHex)
mask = hex2dec(maskHex);
nslot = sum(bitget(mask, 1:16));
c = uint8(hex2dec([{'00','00','99','bb','65'}, {dec2hex(nslot,2)}, ...
    {dec2hex(floor(mask/256),2), dec2hex(mod(mask,256),2)}, ...
    {'00','01','78','00'}, repmat({'00'},1,8)]));
end

function c = mkGet(slotHex, divHex)
c = uint8(hex2dec([{'00','00','99','bb','66'}, {slotHex, '00', divHex}, ...
    {'00','80','00','F0'}, repmat({'00'},1,8)]));
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
    try
        fclose(udp_obj);
    catch
    end
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
