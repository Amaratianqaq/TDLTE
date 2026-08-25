function logResult(script, kv)
% logResult - 记录一次测试运行结果 (追加到 run_log.txt, UTF-8)
% 用法:
%   logResult('verify_rho', struct('rho',0.102,'crc',0))  % 记录一次运行
%   logResult('clear')                                     % 清空全部历史记录
% 日志位置: 与本函数同目录 (Srcs) 下的 run_log.txt
%
% 格式: [时间戳] | 脚本名 | key=value | ...

p = fullfile(fileparts(mfilename('fullpath')), 'run_log.txt');

if ischar(script) && strcmpi(script, 'clear')
    if exist(p, 'file'), delete(p); end
    fprintf('[logResult] 已清空日志: %s\n', p);
    return;
end

ts = datestr(now, 'yyyy-mm-dd HH:MM:SS');
s = sprintf('%s | %s', ts, script);

if nargin >= 2 && isstruct(kv)
    flds = fieldnames(kv);
    for i = 1:numel(flds)
        v = kv.(flds{i});
        if ischar(v)
            s = sprintf('%s | %s=%s', s, flds{i}, v);
        elseif islogical(v)
            s = sprintf('%s | %s=%d', s, flds{i}, v);
        elseif isnumeric(v) && isscalar(v)
            s = sprintf('%s | %s=%.6g', s, flds{i}, v);
        elseif isnumeric(v)
            s = sprintf('%s | %s=%s', s, flds{i}, mat2str(v));
        else
            s = sprintf('%s | %s=%s', s, flds{i}, mat2str(v));
        end
    end
end

fid = fopen(p, 'a', 'n', 'UTF-8');
if fid > 0
    fprintf(fid, '%s\n', s);
    fclose(fid);
else
    warning('logResult: 无法写入日志 %s', p);
end
end
