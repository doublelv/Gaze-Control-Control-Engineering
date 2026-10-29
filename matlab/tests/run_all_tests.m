%RUN_ALL_TESTS  Offline tests (no hardware, no actuators). Run from matlab/tests.
%   Works in MATLAB and GNU Octave. Prints PASS/FAIL per test and errors at the
%   end if anything failed (so CI fails too).
clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'scripts'));
setup_paths();
addpath(here);
setenv('GCCE_NO_LOCAL', '1');                 % defaults only: ignore config/local/params_local.m
cleanupEnv = onCleanup(@() setenv('GCCE_NO_LOCAL', ''));
d = dir(fullfile(here, 'test_*.m'));
names = sort({d.name});
nFail = 0;
for i = 1:numel(names)
    name = names{i}(1:end-2);
    t0 = tic;
    try
        feval(name);
        fprintf('PASS  %-26s (%.1f s)\n', name, toc(t0));
    catch err
        fprintf('FAIL  %-26s %s\n', name, err.message);
        nFail = nFail + 1;
    end
end
fprintf('%d of %d tests passed\n', numel(names) - nFail, numel(names));
if nFail > 0, error('run_all_tests:failed', '%d test(s) failed', nFail); end
