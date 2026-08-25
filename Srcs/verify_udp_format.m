%% verify_udp_format.m - UDP回传格式全面勘探 (四连测版)
% 新增两个决定性试验:
%   控制组C: 不发tx_command/Send_IQ/数据, 只发Get_IQ抓取
%            -> corr(A1,C)≈corr(A1,A2) 则回传与发射无关(射频TX没通)
%   格式勘探: 除6种4字节布局外, 增加
%            a) 每960字节UDP包剥离头部0~16字节 (设备常带包头!)
%            b) 3字节/样点布局 (freqDiv=0)
%            c) 块状布局: 前一半=全部I, 后一半=全部Q
%            d) 直接16位字(word)级相关 (含倒序)
% 判定:
%   - corr(A1,C)≈corr(A1,A2) -> 发射链路没通, 查硬件/上位机
%   - 某格式rho显著高(>0.3) -> 找到真实格式, 据此修OFDM_RxDataDeal

clear; clc;
addpath(fileparts(mfilename('fullpath')));   % 确保 logResult 可用

SAMPLE_LENGTH = 30720;
B = {'00','00','99','bb'};
cmdSync   = uint8(hex2dec({'00','00','99','bb','69','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdRouter = uint8(hex2dec({'00','00','99','bb','68','00','00','00','00','06','00','00','00','00','00','00','00','00','00','00'}));
cmdDelay  = uint8(hex2dec({'00','00','99','bb','67','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdTx     = uint8(hex2dec([B, '65','02','00','03','00','01','78','00', repmat({'00'},1,8)]));
cmdSend   = uint8(hex2dec({'00','00','99','bb','64','00','00','00','00','00','00','00','00','00','00','00','00','00','00','00'}));
cmdGet    = uint8(hex2dec([B, '66','01','00','01','00','80','00','F0', repmat({'00'},1,8)]));

%% 1) 四段试验: A1/A2=图案A x2, B=图案B, C=控制(不发射)
rng(7); txIA = randi([0 4095],1,SAMPLE_LENGTH); txQA = randi([0 4095],1,SAMPLE_LENGTH);
rng(8); txIB = randi([0 4095],1,SAMPLE_LENGTH); txQB = randi([0 4095],1,SAMPLE_LENGTH);
dataA = packIQ(txIA, txQA, SAMPLE_LENGTH);
dataB = packIQ(txIB, txQB, SAMPLE_LENGTH);
ztA = (double(txIA)-2048) + 1j*(double(txQA)-2048);

fprintf('==== 四连测 ====\n');
[rxA1, ok1] = udpLoopback(dataA, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH);
if ~ok1, error('第1次(A1)未收到数据'); end
[rxA2, ok2] = udpLoopback(dataA, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH);
if ~ok2, error('第2次(A2)未收到数据'); end
[rxB,  ok3] = udpLoopback(dataB, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH);
if ~ok3, error('第3次(B)未收到数据'); end
[rxC,  ok4] = udpLoopback([], cmdSync, cmdRouter, cmdDelay, [], [], cmdGet, SAMPLE_LENGTH); % 不发射
if ~ok4, fprintf('注意: 控制组C(不发射)未收到数据\n'); end

save('udp_format_probe.mat', 'rxA1','rxA2','rxB','rxC','dataA','dataB');

%% 2) 统计与相关性
bA1 = double(rxA1); bA2 = double(rxA2); bB = double(rxB);
N = floor(min([numel(bA1), numel(bA2), numel(bB)]) / 4);
bA1 = bA1(1:N*4); bA2 = bA2(1:N*4); bB = bB(1:N*4);

corrB = @(x,y) max(abs(xcorr(x-mean(x), y-mean(y)))) / ...
    sqrt(sum((x-mean(x)).^2) * sum((y-mean(y)).^2));

fprintf('\n==== 结论性指标 ====\n');
fprintf('corr(A1,A2)=%.4f  (同数据两次)\n', corrB(bA1,bA2));
fprintf('corr(A1,B) =%.4f  (不同数据)\n', corrB(bA1,bB));
if ok4
    bC = double(rxC); bC = bC(1:min(numel(bC),N*4));
    cA1C = corrB(bA1, bC);
    fprintf('corr(A1,C) =%.4f  (控制组: 未发射)\n', cA1C);
    if abs(cA1C - corrB(bA1,bA2)) < 0.1
        fprintf('>> 控制组与正常组高度一致 -> 回传与发射内容无关, 发射/射频路径未通!\n');
    else
        fprintf('>> 控制组与正常组差异明显 -> 发射影响了回传, 继续查格式。\n');
    end
else
    fprintf('控制组C(未发射)未收到数据\n');
end

%% 3) 回传A1 字节统计
fprintf('\n==== 回传A1 字节统计 ====\n');
fprintf('min=%d max=%d mean=%.2f std=%.2f 不同取值数=%d\n', ...
    min(bA1), max(bA1), mean(bA1), std(bA1), numel(unique(bA1)));
tbA = double(typecast(dataA,'uint8'));
fprintf('A1前32字节: %s\n', strjoin(arrayfun(@(x) sprintf('%02X',x), rxA1(1:32),'UniformOutput',false),' '));
fprintf('发送A前32字节: %s\n', strjoin(arrayfun(@(x) sprintf('%02X',x), tbA(1:32),'UniformOutput',false),' '));

%% 4) 格式勘探 (全部用A1, 与发送图案A比较)
fprintf('\n==== 格式勘探 (rho) ====\n');
best = struct('name','','rho',0);

% a) 6种4字节分组布局
layouts = {
    '4B I=b0b1,Q=b2b3',   [1 2 3 4];
    '4B I=b2b3,Q=b0b1',   [3 4 1 2];
    '4B I=b1b0,Q=b3b2',   [2 1 4 3];
    '4B I=b3b2,Q=b1b0',   [4 3 2 1];
    '4B I=b0b1,Q=b3b2',   [1 2 4 3];
    '4B I=b1b0,Q=b2b3',   [2 1 3 4];
    };
for k = 1:size(layouts,1)
    p = layouts{k,2};
    o = (0:N-1)*4;
    z = decode4(bA1, p, N);
    rho = rhoMax(z, ztA);
    fprintf('%-22s rho=%.4f\n', layouts{k,1}, rho);
    if rho > best.rho, best = struct('name',layouts{k,1},'rho',rho); end
end

% b) UDP包剥离头部 (每960字节一个包, 剥离h字节)
fprintf('--- 剥离UDP包头扫描 (每包960字节) ---\n');
PKT = 960;
nPkt = floor(numel(bA1)/PKT);
for h = 0:16
    bb = [];
    for p = 1:nPkt
        seg = bA1((p-1)*PKT+1 : p*PKT);
        bb = [bb, seg(h+1:end)];   %#ok<AGROW>
    end
    if numel(bb) < N*4, continue; end
    z = decode4(bb(1:N*4), [1 2 3 4], N);
    rho = rhoMax(z, ztA);
    fprintf('剥离%2d字节: rho=%.4f\n', h, rho);
    if rho > best.rho, best = struct('name',sprintf('剥离包头%dB',h),'rho',rho); end
end

% c) 3字节/样点布局 (freqDiv=0: I=b0<<4|b1>>4, Q=(b1&15)<<8|b2)
z3 = decode3(bA1, N);
rho3 = rhoMax(z3, ztA);
fprintf('3字节/样点布局      : rho=%.4f\n', rho3);
if rho3 > best.rho, best = struct('name','3字节布局','rho',rho3); end

% d) 块状布局: 前N字=I, 后N字=Q (每样点2字节)
w = double(typecast(uint8(bA1),'uint16'));   % 61440个字
w = w - 32768;                                 % 转有符号
zBlk = (w(1:N) - 2048) + 1j*(w(N+1:2*N) - 2048);   % 直接当12bit量值(不右移)
rhoBlk = rhoMax(zBlk, ztA);
fprintf('块状I后Q(16位字)     : rho=%.4f\n', rhoBlk);
if rhoBlk > best.rho, best = struct('name','块状I后Q','rho',rhoBlk); end

% e) 字级直接相关: 回传16位字流 vs 发送字流(错位扫描每样点1字)
tw = double(typecast(dataA,'uint16'));
zW = (w(1:2*N) - 32768);
zT = (tw(1:2*N) - 32768);
cW = xcorr(zW-mean(zW), zT-mean(zT));
rhoW = max(abs(cW)) / sqrt(sum((zW-mean(zW)).^2) * sum((zT-mean(zT)).^2));
fprintf('字流直接相关(含错位)  : rho=%.4f\n', rhoW);
if rhoW > best.rho, best = struct('name','字流直接','rho',rhoW); end

% f) 倒序
zRev = decode4(bA1(end-N*4+1:end), [1 2 3 4], N);
rhoRev = rhoMax(zRev, ztA);
fprintf('4B布局倒序            : rho=%.4f\n', rhoRev);
if rhoRev > best.rho, best = struct('name','倒序','rho',rhoRev); end

fprintf('\n>> 最佳: %s (rho=%.4f)\n', best.name, best.rho);
if best.rho > 0.3
    fprintf('   找到真实格式, 按此修改OFDM_RxDataDeal。\n');
else
    fprintf('   全部格式均不匹配 -> 回传不含我们的数据(发射未通/设备自产内容), 查硬件。\n');
end

%% 记录结果到日志
if exist('cA1C','var'), cA1Cv = cA1C; else, cA1Cv = NaN; end
logResult('verify_udp_format', struct('corrA1A2',cA1A2,'corrAB',cAB,'corrA1C',cA1Cv, ...
    'bestRho',best.rho,'bestName',best.name));

%% ================= 工具函数 =================
function dataIQ = packIQ(txI, txQ, SAMPLE_LENGTH)
len = SAMPLE_LENGTH * 2;
dataIQ = zeros(1, len);
dataIQ(1,1:2:len-1) = txI;
dataIQ(1,2:2:len)   = txQ;
dataIQ(1,1:2:len-1) = dataIQ(1,1:2:len-1) .* 16;
dataIQ(1,2:2:len)   = fix(dataIQ(1,2:2:len)./256) + rem(dataIQ(1,2:2:len),256).*256;
dataIQ = uint16(dataIQ);
end

function z = decode4(b, p, N)
o = (0:N-1)*4;
iw = b(o+p(1)) + b(o+p(2))*256;
qw = b(o+p(3)) + b(o+p(4))*256;
iv = floor(iw/16); qv = floor(qw/16);
iv(iv>2047) = iv(iv>2047) - 4096;
qv(qv>2047) = qv(qv>2047) - 4096;
z = iv + 1j*qv;
end

function z = decode3(b, N)
b = b(1:N*3);
z = zeros(1,N);
for n = 1:N
    o = (n-1)*3;
    i = b(o+1)*16 + floor(b(o+2)/16);
    q = mod(b(o+2),16)*256 + b(o+3);
    if i > 2047, i = i - 4096; end
    if q > 2047, q = q - 4096; end
    z(n) = i + 1j*q;
end
end

function rho = rhoMax(z, zt)
z = z - mean(z);
zt = zt - mean(zt);
c = xcorr(z, zt);
rho = max(abs(c)) / (sqrt(sum(abs(z).^2)) * sqrt(sum(abs(zt).^2)));
end

function [rxBytes, ok] = udpLoopback(dataIQ, cmdSync, cmdRouter, cmdDelay, cmdTx, cmdSend, cmdGet, SAMPLE_LENGTH)
pcip   = '192.168.1.180';
xsrpip = '192.168.1.166';
udp_obj = udp(xsrpip,13344,'LocalHost',pcip,'LocalPort',12345,'TimeOut',15, ...
    'OutputBufferSize',61440,'InputBufferSize',61440*10);
udp_obj.Timeout = 15;
ok = false; rxBytes = uint8([]);
try
    fopen(udp_obj);
    fwrite(udp_obj, cmdSync,   'uint8');
    fwrite(udp_obj, cmdRouter, 'uint8');
    fwrite(udp_obj, cmdDelay,  'uint8');
    if ~isempty(cmdTx)
        fwrite(udp_obj, cmdTx, 'uint8');
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
        rxBytes = uint8(buf(1:data_byte_c));
        ok = true;
    end
catch ME
    fprintf('    UDP错误: %s\n', ME.message);
end
try, fclose(udp_obj); catch, end
delete(udp_obj); clear udp_obj;
end
