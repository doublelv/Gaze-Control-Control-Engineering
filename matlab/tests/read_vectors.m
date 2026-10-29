function V = read_vectors()
%READ_VECTORS  Parse interface/test-vectors/protocol_v1.txt into a struct array
%   with fields name, kind, f (struct of numeric fields), bytes (uint8 row).
here = fileparts(mfilename('fullpath'));
cands = {fullfile(here, '..', '..', 'interface', 'test-vectors', 'protocol_v1.txt'), ...
         fullfile(here, 'protocol_v1.txt')};                % standalone copy
f = '';
for i = 1:numel(cands), if exist(cands{i}, 'file'), f = cands{i}; break, end, end
assert(~isempty(f), 'protocol_v1.txt not found');
txt = strsplit(fileread(f), sprintf('\n'));
V = struct('name', {}, 'kind', {}, 'f', {}, 'bytes', {});
for k = 1:numel(txt)
    L = strtrim(txt{k});
    if isempty(L) || L(1) == '#', continue, end
    c = strtrim(strsplit(L, '|'));
    v.name = c{1}; v.kind = c{2}; v.f = struct();
    if ~strcmp(c{3}, '-')
        kv = strsplit(c{3}, ' ');
        for j = 1:numel(kv)
            if isempty(kv{j}), continue, end
            p = strsplit(kv{j}, '=');
            v.f.(p{1}) = str2double(p{2});
        end
    end
    v.bytes = uint8(hex2dec(strsplit(c{4}, ' ')))';
    V(end+1) = v; %#ok<AGROW>
end
end
