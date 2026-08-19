function grid = ResMap(dataSyms, rs0, rs1, prm)
% ResMap - 资源映射, 将数据符号和导频映射到资源网格 (3GPP 36.211 5.3.2.3)
% 输入:
%   dataSyms - 数据复数符号列向量 (长度 = NSubc x 数据符号数 = 1200x12)
%   rs0, rs1 - 导频符号0/符号1的DMRS序列 (NSubc x 1)
%   prm      - 参数结构体 (含 NSubc, NSym, PilotSymbols)
% 输出:
%   grid - NSubc x NSym 复数矩阵 (资源网格, 其余位置为0)
%
% 映射规则 (文档2.2.11 / 36.211 5.3.2.3):
%   - 网格 NSubc(1200) x NSym(14), 导频占据符号3和10 (文档L73)
%   - 数据符号 z(0)..z(Msymb-1) 从z(0)开始, 按"先增加k(频率)再增加l(时间)"
%     依次填入非导频符号, 导频位置不填数据
%   - 数据符号顺序: 0,1,2,4,5,6,7,8,9,11,12,13 (共12个)

grid = zeros(prm.NSubc, prm.NSym);

% 1) 导频映射 (符号3、10)
grid(:, prm.PilotSymbols(1) + 1) = rs0(:);
grid(:, prm.PilotSymbols(2) + 1) = rs1(:);

% 2) 数据映射: 先k后l, 跳过导频符号
idx = 1;
for l = 0:prm.NSym - 1
    if any(prm.PilotSymbols == l)
        continue;                        % 导频符号, 跳过
    end
    grid(:, l + 1) = dataSyms(idx : idx + prm.NSubc - 1);
    idx = idx + prm.NSubc;
end
end
