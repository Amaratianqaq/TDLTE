%% analyze_udp_probe.m - 离线分析 udp_format_probe.mat (不需要硬件)
% 目标: 从真实回传字节数据中找到XSRP的IQ格式 (决定 OFDM_RxDataDeal 怎么写)
% 数据来源: verify_udp_format.m 保存的 rxA1/rxA2/rxB/rxC/dataA/dataB
% 方法:
%   1) 控制组判断: 回传是否与发射内容相关
%   2) 字节级对比: 回传A1 vs 发送A 的前32字节
%   3) 全面格式枚举: 4字节样点(字序x解码x交叉x倒序x包头偏移) + 3字节样点

clear; clc;
load('udp_format_probe.mat');

%% 1) 基本统计与控制组
bA1 = double(rxA1); bA2 = double(rxA2); bB = double(rxB);
N = floor(min([numel(bA1), numel(bA2), numel(bB)]) / 4);
bA1 = bA1(1:N*4); bA2 = bA2(1:N*4); bB = bB(1:N*4);

corrB = @(x,y) max(abs(xcorr(x-mean(x), y-mean(y)))) / ...
    sqrt(sum((x-mean(x)).^2) * sum((y-mean(y)).^2));
fprintf('==== 控制组判断 ====\n');
cA1A2 = corrB(bA1, bA2); cAB = corrB(bA1, bB);
fprintf('corr(A1,A2)=%.4f (同数据两次), corr(A1,B)=%.4f (不同数据)\n', cA1A2, cAB);
if exist('rxC','var') && ~isempty(rxC) && numel(rxC) >= 4
    bC = double(rxC); bC = bC(1:min(numel(bC), N*4));
    cA1C = corrB(bA1, bC);
    fprintf('corr(A1,C)=%.4f (控制组: 未发射) -> ', cA1C);
    if abs(cA1C - cA1A2) < 0.1
        fprintf('回传与发射内容无关, 发射/射频路径未通!\n');
    else
        fprintf('回传受发射影响, 继续查格式\n');
    end
end

%% 2) 字节级对比 (回传A1 vs 发送A)
tbA = double(typecast(dataA, 'uint8'));
fprintf('\n==== 字节级对比 (前32字节) ====\n');
fprintf('回传A1: %s\n', strjoin(arrayfun(@(x) sprintf('%02X',x), bA1(1:32),'UniformOutput',false),' '));
fprintf('发送A : %s\n', strjoin(arrayfun(@(x) sprintf('%02X',x), tbA(1:32),'UniformOutput',false),' '));

%% 3) 重建发送的原始IQ (从打包数据反推)
dataAd = double(dataA);
iw = dataAd(1:2:end); qw = dataAd(2:2:end);
txI12 = floor(iw/16);
txQ12 = bitshift(qw, -8) + bitand(qw, 15)*256;   % Q字节交换逆
txI12(txI12 > 2047) = txI12(txI12 > 2047) - 4096;
txQ12(txQ12 > 2047) = txQ12(txQ12 > 2047) - 4096;
ztA = (txI12 - 2048) + 1j*(txQ12 - 2048);        % 与 verify_udp_format 的参考一致

%% 4) 全面格式枚举
fprintf('\n==== 格式枚举 (rho 对 ztA) ====\n');
best = struct('name','','rho',0);
N = min(N, numel(ztA));

% 4.1) 4字节/样点: 字序组合 × Q解码方式 × I/Q位置 × 倒序
byteOrders = {'LE','BE'};
for io = 1:2              % I 字序
    for qo = 1:2          % Q 字序
        for sw = 0:1      % I/Q 对调
            for rev = 0:1 % 整体倒序
                b = bA1;
                if rev, b = b(end:-1:1); end
                for qmode = 1:4
                    % qmode 1: Q直接>>4 (tone脚本/decode4假设)
                    % qmode 2: Q字节交换逆 (OFDM_RxDataDeal旧假设)
                    % qmode 3: Q取低12位不右移 (块状假设)
                    % qmode 4: I与Q均字节交换逆 (设备实测格式, 2026-08-27确认)
                    z = zeros(1, N);
                    for n = 0:N-1
                        o = n*4;
                        if sw == 0
                            iw_ = w2num(b(o+1), b(o+2), byteOrders{io});
                            qw_ = w2num(b(o+3), b(o+4), byteOrders{qo});
                        else
                            iw_ = w2num(b(o+3), b(o+4), byteOrders{qo});
                            qw_ = w2num(b(o+1), b(o+2), byteOrders{io});
                        end
                        switch qmode
                            case 1, iv = floor(iw_/16); qv = floor(qw_/16);
                            case 2, iv = floor(iw_/16); qv = bitshift(qw_,-8) + bitand(qw_,15)*256;
                            case 3, iv = iw_ - 2048;    qv = qw_ - 2048;
                            case 4, iv = bitshift(iw_,-8) + bitand(iw_,15)*256; qv = bitshift(qw_,-8) + bitand(qw_,15)*256;
                        end
                        if iv > 2047, iv = iv - 4096; end
                        if qv > 2047, qv = qv - 4096; end
                        z(n+1) = iv + 1j*qv;
                    end
                    rho = rhoMax(z, ztA(1:N));
                    nm = sprintf('%s%s I%s Q%s %s', byteOrders{io}, byteOrders{qo}, ...
                        ternary(sw,'=Q','=I'), ...
                        ternary(sw,'=I','=Q'), ...
                        ternary(rev,'倒序','') );
                    if rho > best.rho, best = struct('name',nm,'rho',rho); end
                    if rho > 0.3
                        fprintf('  rho=%.4f  %s (qmode=%d)\n', rho, nm, qmode);
                    end
                end
            end
        end
    end
end

% 4.2) UDP包头剥离扫描 (每960字节, 剥0~16字节; 用当前OFDM_RxDataDeal格式)
fprintf('--- 包头剥离 + OFDM_RxDataDeal格式 ---\n');
PKT = 960; nPkt = floor(numel(bA1)/PKT);
for h = 0:16
    bb = [];
    for p = 1:nPkt
        seg = bA1((p-1)*PKT+1 : p*PKT);
        bb = [bb, seg(h+1:end)]; %#ok<AGROW>
    end
    if numel(bb) < N*4, continue; end
    b2 = bb(1:N*4);
    z = decode4(b2, N, 2);    % qmode=2: Q字节交换逆
    rho = rhoMax(z, ztA(1:N));
    if rho > 0.3 || h <= 3
        fprintf('  剥%2d字节: rho=%.4f\n', h, rho);
    end
    if rho > best.rho, best = struct('name',sprintf('剥包头%dB',h),'rho',rho); end
end

% 4.3) 3字节/样点 (freqDiv=0 布局)
z3 = decode3(bA1, N);
rho3 = rhoMax(z3, ztA(1:N));
fprintf('3字节/样点: rho=%.4f\n', rho3);
if rho3 > best.rho, best = struct('name','3字节','rho',rho3); end

fprintf('\n>> 最佳格式: %s (rho=%.4f)\n', best.name, best.rho);
if best.rho > 0.3
    fprintf('  找到真实格式! 按此修改 OFDM_RxDataDeal。\n');
else
    fprintf('  全部格式均不匹配 -> 回传不含我们的数据, 查发射/射频链路(频率配置等)。\n');
end

%% ================= 工具函数 =================
function v = w2num(b0, b1, order)
if strcmp(order, 'LE')
    v = b0 + b1*256;
else
    v = b1 + b0*256;
end
end

function z = decode4(b, N, qmode)
z = zeros(1, N);
for n = 0:N-1
    o = n*4;
    iw_ = b(o+1) + b(o+2)*256;
    qw_ = b(o+3) + b(o+4)*256;
    switch qmode
        case 1, iv = floor(iw_/16); qv = floor(qw_/16);
        case 2, iv = floor(iw_/16); qv = bitshift(qw_,-8) + bitand(qw_,15)*256;
    end
    if iv > 2047, iv = iv - 4096; end
    if qv > 2047, qv = qv - 4096; end
    z(n+1) = iv + 1j*qv;
end
end

function z = decode3(b, N)
b = b(1:N*3);
z = zeros(1, N);
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

function r = ternary(c, a, b)
if c, r = a; else, r = b; end
end
