function test_offline_margin2()
% test_offline_margin2 - 信道估计时域门控测试 (无需硬件)
% 依据: echo_check 曾测得信道冲激响应 CP外能量占75.5%。若那是LS噪声,
%       将信道估计IFFT后只保留CP内径(其余置0)再FFT回来, 可显著降噪。
% 变体:
%   V5 时域门控信道估计(保留CP内径) + 排除污染行噪声
%   V6 V5 + 每子载波LLR加权
%   V7 门控长度扫描 (64/144/160/288/320/1024 样点)
% 输出: 各变体 BER/CRC + 信道冲激响应统计 (验证CP外能量成分)

clear; clc; close all;
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Tx'));
addpath(fullfile(fileparts(mfilename('fullpath')), 'Rx'));

if ~exist('rx_accum_v3_result.mat', 'file')
    error('没有 rx_accum_v3_result.mat, 请先运行 rx_accum_v3');
end
S = load('rx_accum_v3_result.mat');
Davg = S.Davg(:).';
prm = GetLTEParams();

rng(42);
[txBits, ~] = TBGen(prm); txBits = txBits(:);

rxGrid = OFDMDemod(Davg, prm);
[rs0, rs1] = RefSignalGen(prm);
Hraw = ChanEst(rxGrid, rs0, rs1, prm);

notchMask = false(prm.NSubc, 1); notchMask(451:750) = true;
clean = ~notchMask;
pilotIdx1 = prm.PilotSymbols(1)+1; pilotIdx2 = prm.PilotSymbols(2)+1;

% 参考: 频域平滑(原链)
Hsm = Hraw;
for l = 1:prm.NSym
    h = Hsm(:,l); h(notchMask) = NaN;
    h = movmean(h, 64, 'omitnan');
    h(notchMask) = Hsm(notchMask,l);
    Hsm(:,l) = h;
end

% 信道冲激响应统计 (导频列, 干净行, IFFT)
Hp = Hsm(:, pilotIdx1); Hp(~clean) = 0;
hIR = ifft(Hp, 1200);
p = abs(hIR).^2; ptot = sum(p);
CP_pts = round(144 / 32.55);   % CP=4.69us / 55.6ns每点
eIn = 100*sum(p(1:CP_pts))/ptot; eOut = 100*sum(p(CP_pts+1:end))/ptot;
fprintf('信道冲激响应: CP内(%.1fus)能量=%.1f%% | CP外=%.1f%%\n', CP_pts*0.0556, eIn, eOut);
% CP外最强径位置
[pm, im] = max(p(CP_pts+1:end));
fprintf('CP外最强径: %.1fus处, 相对主径 %.1f dB\n', (CP_pts+im)*0.0556, 10*log10(pm/max(p)+eps));

% 时域门控函数: H -> IFFT -> 保留前Ntaps -> FFT
gateH = @(H, Ntaps) real(ifft(fft(H) .* [ones(Ntaps,1); zeros(1200-Ntaps,1)])) ...
    + 1j*imag(ifft(fft(H) .* [ones(Ntaps,1); zeros(1200-Ntaps,1)]));

% 发送端segInfo
rng(42);
[bits, info] = TBGen(prm);
bitsCRC = CRCAttach(bits, '24A');
[~, segInfo] = CodeBlockSeg(bitsCRC);

% 解码器 (与test_offline_margin相同)
    function [ber, crc] = runDecode(rxLLR)
        descrambled = DescrambleSoft(rxLLR, prm);
        Nsc = prm.NSubc; NsymData = prm.NSym - prm.NPilot; Qm = prm.Qm;
        G = Nsc * NsymData * prm.NL * Qm;
        C = segInfo.C; Kp = segInfo.Kp; Km = segInfo.Km; Cp = segInfo.Cp; Cm = segInfo.Cm;
        Krs = [repmat(Km, Cm, 1); repmat(Kp, Cp, 1)];
        cbDecoded = cell(1, C);
        ptr = 1;
        for r = 1:C
            E = prm.NL * Qm * floor(G / (prm.NL * Qm * C));
            cbBits = descrambled(ptr:ptr+E-1);
            ptr = ptr + E;
            rmOut = DeRateMatch(cbBits, Krs(r), C, G, Qm, prm.NL, prm.RV);
            [cbDecoded{r}, ~] = TurboDecode(rmOut, Krs(r), prm.maxIter);
        end
        [bitsOut, crc] = CodeBlockDeSeg(cbDecoded, segInfo);
        b = min(numel(bitsOut), numel(txBits));
        ber = mean(bitsOut(1:b) ~= txBits(1:b));
    end

% ============ 变体 ============
fprintf('\n==== 时域门控测试 ====\n');
Nsc = prm.NSubc; NsymData = prm.NSym - prm.NPilot; Qm = prm.Qm;

for Ntaps = [32 64 144 160 288 320 512 1024]
    Hg = zeros(size(Hsm));
    for l = 1:prm.NSym
        hc = Hsm(:, l);
        hc(~clean) = 0;                    % 污染行先置0再门控
        Hg(:, l) = gateH(hc, Ntaps);
        Hg(~clean, l) = Hsm(~clean, l);    % 污染行恢复
    end
    resK = 0.5*(abs(rxGrid(:,pilotIdx1) - Hg(:,pilotIdx1).*rs0).^2 + ...
                abs(rxGrid(:,pilotIdx2) - Hg(:,pilotIdx2).*rs1).^2);
    noiseVar = max(median(resK(clean)), 1e-9);
    dataSyms = ExtractData(Equalize(rxGrid, Hg, 'MMSE', noiseVar), prm);
    rxLLR = DemodulateSoft(dataSyms, prm.Modulation, noiseVar);
    % 每子载波SNR
    txRef = zeros(size(dataSyms));
    [ber, crc] = runDecode(rxLLR);
    fprintf('门控Ntaps=%4d: BER=%.4f CRC=%d\n', Ntaps, ber, crc);
end

fprintf('\n参考: 未门控(频域平滑64)基线 BER=0.2844 CRC=0 (来自test_offline_margin V1)\n');

end
