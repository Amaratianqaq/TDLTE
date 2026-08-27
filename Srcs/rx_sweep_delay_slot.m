function rx_sweep_delay_slot()
% rx_sweep_delay_slot - 好状态下重扫 0x67 延迟字 + Get_IQ 采集时隙 + TX时隙掩码
% 背景: 这些参数曾在坏状态(rho~0.02)扫过无果; 现在好状态(直连/1GHz, 基线rho=0.30)重扫。
% 目标: 找 ①rho(A,X)更高的配置 ②控制帧不再含回环(rho(C,X)低)的"真接收"配置。
% 阶段:
%   P1 0x67延迟字(BE16bit)   P2 采集停顿(pause)   P3 Get_IQ时隙字节
%   P4 TX时隙掩码x采集时隙   P5 最佳配置 K=8 相干平均 + CRC
% 前置: 直连+1GHz+衰减2/增益5+AC耦合+真实系统, 勿碰设备。
% 使用: 直接运行, 约8~12分钟。

addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec([B, '68','00','00','00','00','06','00','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
pcip = '192.168.1.180'; xsrpip = '192.168.1.166';

prm = GetLTEParams();
rng(42); txX = OFDM_TxMain(prm); txX = txX(:).';
dataX = packIQ(txX, SAMPLE_LENGTH);

allResults = cell(0,1);
bestScore = -1; bestCfg = '';

%% ========== 指标评估 (嵌套函数, 共享工作区) ==========
    function out = evalCfg(tag, cmdDelay, cmdTx, cmdGet, pauseSec)
        rxA = udpLoopback(dataX, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip, pauseSec);
        rxC = udpLoopback([],   cmdSync, cmdRouter, cmdDelay, cmdTx, [],       cmdGet, SAMPLE_LENGTH, pcip, xsrpip, pauseSec);
        out = struct('tag',tag,'rhoAX',NaN,'rhoCX',NaN,'corrAC',NaN,'rhoD',NaN,'removed',NaN,'outPct',NaN);
        if isempty(rxA) || isempty(rxC), return; end
        rxA = rxA(:).'; rxC = rxC(:).';
        [~, out.rhoAX, ~, ~] = syncLTE(rxA, txX, prm);
        [~, out.rhoCX, ~, ~] = syncLTE(rxC, txX, prm);
        out.corrAC = max(abs(xcorr(rxA-mean(rxA), rxC-mean(rxC)))) / ...
            (sqrt(sum(abs(rxA-mean(rxA)).^2)) * sqrt(sum(abs(rxC-mean(rxC)).^2)));
        cX = xcorr(rxA - mean(rxA), rxC - mean(rxC));
        [~, imax] = max(abs(cX));
        C_al = circshift(rxC, -(imax - (numel(cX)+1)/2));
        alpha = sum(conj(C_al).*rxA) / sum(abs(C_al).^2);
        D = rxA - alpha*C_al;
        [~, out.rhoD, ~, ~] = syncLTE(D, txX, prm);
        out.removed = (1 - sum(abs(D).^2)/sum(abs(rxA).^2)) * 100;
        Xf = fft(rxA); f = (0:SAMPLE_LENGTH-1)/SAMPLE_LENGTH*30.72e6; f(f>15.36e6) = f(f>15.36e6)-30.72e6;
        P = abs(Xf).^2;
        out.outPct = 100*sum(P(abs(f)>9e6)) / sum(P);
        fprintf('%-22s | rho(A,X)=%.4f | rho(C,X)=%.4f | corr(A,C)=%.4f | rho(D,X)=%.4f | 消能%.1f%% | >9M%.1f%%\n', ...
            tag, out.rhoAX, out.rhoCX, out.corrAC, out.rhoD, out.removed, out.outPct);
        score = out.rhoAX + (out.rhoCX < 0.15)*0.2 + (out.corrAC < 0.4)*0.1;
        allResults{end+1} = out; %#ok<AGROW>
        if score > bestScore && ~isnan(out.rhoAX)
            bestScore = score; bestCfg = tag;
        end
    end

%% ========== P1: 0x67 延迟字 ==========
fprintf('==== P1: 0x67 延迟字 (BE16, 停顿0) ====\n');
for d = [0 1 2 4 8 16 32 64 128 256 512 1024 2048 4096 8192 16384 30000 65535]
    evalCfg(sprintf('delay=%d', d), mkDelay(d), mkTx('0003'), mkGet('01','01'), 0);
end

%% ========== P2: 采集停顿 ==========
fprintf('\n==== P2: 采集停顿 (延迟字=0) ====\n');
for ps = [0.2 0.5 1.0 2.0]
    evalCfg(sprintf('pause=%.1fs', ps), mkDelay(0), mkTx('0003'), mkGet('01','01'), ps);
end

%% ========== P3: Get_IQ 采集时隙字节 ==========
fprintf('\n==== P3: Get_IQ 时隙字节 (byte5) ====\n');
for sl = {'00','01','02','03','04','05'}
    evalCfg(sprintf('slot=%s', sl{1}), mkDelay(0), mkTx('0003'), mkGet(sl{1},'01'), 0);
end

%% ========== P4: TX时隙掩码 x 采集时隙 ==========
fprintf('\n==== P4: TX掩码 x 采集时隙 ====\n');
for mask = {'0001','0003','000F'}
    for sl = {'00','01','03'}
        evalCfg(sprintf('tx=%s/slot=%s', mask{1}, sl{1}), mkDelay(0), mkTx(mask{1}), mkGet(sl{1},'01'), 0);
    end
end

%% ========== P5: 最佳配置 K=8 相干平均 + CRC ==========
fprintf('\n==== P5: 最佳配置 [%s] K=8 相干平均 + CRC ====\n', bestCfg);
bestDelay = 0; bestMask = '0003'; bestSlot = '01'; pauseB = 0;
if contains(bestCfg, 'delay=')
    bestDelay = str2double(extractAfter(bestCfg, 'delay='));
end
if contains(bestCfg, 'pause=')
    t = extractAfter(bestCfg, 'pause=');
    pauseB = str2double(t(1:end-1));   % 去掉末尾的 's'
end
if contains(bestCfg, 'slot=')
    t = extractAfter(bestCfg, 'slot=');
    if numel(t) == 2, bestSlot = t; end
end
if contains(bestCfg, 'tx=')
    t = extractBetween(bestCfg, 'tx=', '/');
    if ~isempty(t), bestMask = t{1}; end
end

if isempty(bestCfg)
    fprintf('>> 全部配置未收到数据, 跳过P5\n');
    Davg = []; rhoAvg = NaN; snrPer = []; crcOK = NaN;
else
sumAcc = zeros(1, SAMPLE_LENGTH);
rhos = zeros(1, 8);
for k = 1:8
    rxA = udpLoopback(dataX, cmdSync, cmdRouter, mkDelay(bestDelay), mkTx(bestMask), cmdSend, mkGet(bestSlot,'01'), SAMPLE_LENGTH, pcip, xsrpip, pauseB);
    if isempty(rxA), fprintf('第%d帧未收到\n', k); continue; end
    [rs, rho, ~, ~, ph] = syncLTE(rxA(:).', txX, prm);
    sumAcc = sumAcc + rs .* exp(-1j*ph);
    rhos(k) = rho;
    fprintf('第%d帧 rho=%.4f\n', k, rho);
end
Davg = sumAcc / 8;
[~, rhoAvg] = syncLTE(Davg, txX, prm);
fprintf('K=8平均 rho=%.4f (单帧均值=%.4f)\n', rhoAvg, mean(rhos));
snrPer = subcarrierSNR(Davg, txX, prm);
fprintf('每子载波SNR: 平均=%.2f dB | 中位=%.2f dB | 90分位=%.2f dB | >0dB占比=%.1f%%\n', ...
    mean(snrPer), median(snrPer), quantile(snrPer,0.9), 100*mean(snrPer>0));
try
    [~, crcOK] = OFDM_RxMain(Davg, prm);
    fprintf('CRC=%d\n', crcOK);
catch ME
    fprintf('CRC出错: %s\n', ME.message);
    crcOK = NaN;
end
end   % if isempty(bestCfg)

%% 汇总
if isempty(allResults)
    fprintf('>> 没有收到任何配置的数据\n');
    return;
end
Sall = [allResults{:}];
fprintf('\n==== 汇总: 前5名 (按rhoAX) ====\n');
[~, ix] = sort([Sall.rhoAX], 'descend');
for j = 1:min(5, numel(ix))
    r = Sall(ix(j));
    fprintf('%s : rho(A,X)=%.4f rho(C,X)=%.4f corr(A,C)=%.4f\n', r.tag, r.rhoAX, r.rhoCX, r.corrAC);
end
fprintf('>> 最佳: %s\n', bestCfg);

save('rx_sweep_delay_slot_result.mat', 'allResults', 'bestCfg', 'Davg', 'rhoAvg', 'snrPer', 'crcOK');
try
    logResult('rx_sweep_delay_slot', struct('bestCfg',bestCfg,'rhoAvg',rhoAvg, ...
        'snrAvg',mean(snrPer),'crc',crcOK));
catch
end

end

%% ================= 局部函数 =================
function c = mkDelay(d)
% 0x67 + BE16延迟字 + 13个0
c = uint8(hex2dec([{'00','00','99','bb','67'}, {dec2hex(floor(d/256),2), dec2hex(mod(d,256),2)}, repmat({'00'},1,13)]));
end

function c = mkTx(maskHex)
% 0x65 + 时隙数 + 掩码BE + 分频0001 + 数据个数7800 + 8个0
mask = hex2dec(maskHex);
nslot = sum(bitget(mask, 1:16));
c = uint8(hex2dec([{'00','00','99','bb','65'}, {dec2hex(nslot,2)}, ...
    {dec2hex(floor(mask/256),2), dec2hex(mod(mask,256),2)}, ...
    {'00','01','78','00'}, repmat({'00'},1,8)]));
end

function c = mkGet(slotHex, divHex)
% 0x66 + 时隙 + 00 + 分频 + 包数0080 + 包大小00F0 + 8个0
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

function rxIQ = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH, pcip, xsrpip, pauseSec)
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
    if pauseSec > 0, pause(pauseSec); end
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
        iv = floor(iw/16);
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
