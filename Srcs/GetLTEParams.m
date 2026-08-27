function prm = GetLTEParams()
% GetLTEParams - TD-LTE物理层(PUSCH结构)全局参数配置
% 输出:
%   prm - 结构体, 包含所有系统参数
%
% 与文档(项目任务书 2.2节)对应关系:
%   - 子载波数 1200 (L91: subcarryrnum=1200)  -> NRB=100, NFFT=2048
%   - 每子帧14个OFDM符号, 导频占符号3和10 (L73)
%   - 调制方式 QPSK/16QAM/64QAM 可设置 (L21), TBsize: QPSK=15840,
%     16QAM=28336, 64QAM=55056 (L628)
%   - 正常CP: 符号0 CP=160, 其余符号 CP=144 -> 每子帧
%     14*2048 + 160 + 13*144 = 30720 样点 (与OFDM_RFLoopback一致)

%% ===== 频域/时域基本参数 =====
prm.NRB        = 100;                 % 资源块数 (100RB = 20MHz)
prm.NSubc      = prm.NRB * 12;        % 子载波数 = 1200
prm.NDataSubc  = prm.NSubc - 1;       % DC置空后数据子载波数 = 1199 (符合LTE: DC不承载数据)
prm.DCIdx      = prm.NSubc / 2 + 1;   % DC子载波行号 = 601 (置空)
prm.NFFT       = 2048;                % IFFT/FFT 点数
prm.NSym       = 14;                  % 每子帧 OFDM 符号数
prm.PilotSymbols = [3 10];            % 导频符号位置 (0起, 文档L73)
prm.NPilot     = numel(prm.PilotSymbols);  % 导频符号个数 = 2
prm.NSlot      = 0;                   % 时隙号 (0/1), 由上层随时间推进

%% ===== 小区/用户参数 =====
prm.CellID     = 0;                   % 物理层小区标识 N_ID^cell (0~503)
prm.RNTI       = 1;                   % 用户标识 (16bit)
prm.DeltaSS    = 0;                   % 序列移位样式参数 Δss (0~29)
prm.NDMRS1     = 0;                   % 广播配置 n_DMRS^(1) (0~7), 对应文档 ndmrs1
prm.CyclicShift = 0;                  % DCI调度 n_DMRS^(2) (0~7), 对应文档 cyc_shift
prm.GroupHopping   = 'Off';          % 序列组跳转开关 ('Off'/'On')
prm.SequenceHopping = 'Off';         % 序列跳转开关 ('Off'/'On')

%% ===== 传输参数 =====
prm.Modulation = 'QPSK';              % 调制方式: 'QPSK' / '16QAM' / '64QAM'
switch prm.Modulation
    case 'QPSK',  prm.Qm = 2;  prm.TBsize = 15840;
    case '16QAM', prm.Qm = 4;  prm.TBsize = 28336;
    case '64QAM', prm.Qm = 6;  prm.TBsize = 55056;
    otherwise,    error('GetLTEParams: 不支持的调制方式 %s', prm.Modulation);
end
prm.NL         = 1;                   % 传输层数
prm.RV         = 0;                   % 冗余版本 (0~3)
prm.maxIter    = 24;                   % Turbo译码最大迭代次数 (低SNR下更多迭代可降低门槛)

%% ===== 信道估计频域平滑 =====
% LS信道估计在低SNR下噪声大, 对H沿子载波做滑动平均可显著降噪
% (信道随子载波缓变/平坦时有效, 平坦信道收益最大)。
% chanSmooth=子载波窗口(0=关闭); 默认64(≈0.96MHz)兼顾平坦与轻微选择性。
prm.chanSmooth = 64;

%% ===== 频域去干扰 =====
% 设备回传在0~1MHz存在强确定性干扰(本机内容), 污染低频子载波。
% NotchMHz>0: 把 |f|<=NotchMHz 的子载波LLR置0(按擦除处理), 消除该干扰。
% 默认0=关闭! (仿真/白噪声场景下擦除有效子载波会抬高SNR门槛, 勿默认开启)
% 硬件场景确有低频干扰时再显式设 1.0~1.5。Turbo(码率1/3)可容忍~11%擦除。
prm.NotchMHz = 0;

%% ===== 循环前缀(正常CP) =====
% 每时隙7个符号: 符号0的CP=160, 符号1~6的CP=144; 子帧含2个时隙
prm.CPLengths  = [160, repmat(144, 1, 6), 160, repmat(144, 1, 6)];
prm.NSample    = sum(prm.NFFT + prm.CPLengths);       % 每子帧样点数 = 30720

end
