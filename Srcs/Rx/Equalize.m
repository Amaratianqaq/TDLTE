function eqGrid = Equalize(rxGrid, H, method, noiseVar)
% Equalize - 信道均衡
% 输入:
%   rxGrid   - 接收资源网格 (NSubc x NSym)
%   H        - 信道估计 (NSubc x NSym)
%   method   - 'ZF' (迫零) 或 'MMSE' (最小均方误差)
%   noiseVar - 噪声方差 (仅MMSE需要, ZF可忽略)
% 输出:
%   eqGrid - 均衡后资源网格 (NSubc x NSym)
%
% 算法:
%   ZF:   eqGrid = rxGrid ./ H
%   MMSE: eqGrid = rxGrid .* conj(H) ./ (|H|^2 + noiseVar)

switch upper(method)
    case 'ZF'
        eqGrid = rxGrid ./ H;
        
    case 'MMSE'
        if nargin < 4 || isempty(noiseVar)
            error('Equalize: MMSE方法需要提供噪声方差 noiseVar');
        end
        eqGrid = rxGrid .* conj(H) ./ (abs(H).^2 + noiseVar);
        
    otherwise
        error('Equalize: 不支持的均衡方法 %s (仅支持 ZF 或 MMSE)', method);
end

end
