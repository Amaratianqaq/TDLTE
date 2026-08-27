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
NDataSubc = prm.NDataSubc;          % 1199 (DC置空)

% 导频符号位置 (MATLAB 1-indexed)
pilotIdx1 = prm.PilotSymbols(1) + 1;   % 符号3 -> 索引4
pilotIdx2 = prm.PilotSymbols(2) + 1;   % 符号10 -> 索引11

% 提取非导频符号 (共12个), 跳过DC子载波行
dataSyms = zeros(NDataSubc * (NSym - prm.NPilot), 1);
rows = [1:prm.DCIdx-1, prm.DCIdx+1:NSubc];   % 除DC外的物理子载波行
idx = 1;
for l = 1:NSym
    if l == pilotIdx1 || l == pilotIdx2
        continue;                    % 跳过导频符号
    end
    col = eqGrid(:, l);
    dataSyms(idx : idx + NDataSubc - 1) = col(rows);
    idx = idx + NDataSubc;
end

end
