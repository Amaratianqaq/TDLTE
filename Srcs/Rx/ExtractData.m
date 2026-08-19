function dataSyms = ExtractData(eqGrid, prm)
% ExtractData - 从资源网格中提取数据符号
% 输入:
%   eqGrid - 均衡后资源网格 (NSubc x NSym)
%   prm    - 参数结构体 (含 NSubc, NSym, PilotSymbols)
% 输出:
%   dataSyms - 数据复数符号列向量 (长度 = NSubc * 12)
%
% 算法:
%   - 去除导频列 (符号3和10)
%   - 将剩余12列的数据符号按列优先提取为列向量

NSubc = prm.NSubc;                  % 1200
NSym  = prm.NSym;                   % 14

% 导频符号位置 (MATLAB 1-indexed)
pilotIdx1 = prm.PilotSymbols(1) + 1;   % 符号3 -> 索引4
pilotIdx2 = prm.PilotSymbols(2) + 1;   % 符号10 -> 索引11

% 提取非导频符号 (共12个)
dataSyms = zeros(NSubc * (NSym - prm.NPilot), 1);
idx = 1;
for l = 1:NSym
    if l == pilotIdx1 || l == pilotIdx2
        continue;                    % 跳过导频符号
    end
    dataSyms(idx : idx + NSubc - 1) = eqGrid(:, l);
    idx = idx + NSubc;
end

end
