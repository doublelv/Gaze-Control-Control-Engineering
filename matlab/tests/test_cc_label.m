function test_cc_label()
%TEST_CC_LABEL  Connected components: known shapes and 8-connectivity.
m = false(10, 12);
m(2:3, 2:3) = true;          % square
m(5, 5) = true; m(6, 6) = true;   % diagonal pair -> one component (8-conn)
m(8:9, 10:12) = true;        % bar
m(2, 11) = true;             % single pixel
[L, n] = cc_label(m);
assert(n == 4, 'expected 4 components, got %d', n);
assert(numel(unique(L(2:3, 2:3))) == 1 && L(5, 5) == L(6, 6), 'wrong grouping');
assert(all(L(~m) == 0), 'background labelled');
[~, n0] = cc_label(false(5)); assert(n0 == 0, 'empty mask');
end
