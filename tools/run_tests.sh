#!/usr/bin/env bash
# Run all offline tests locally (same as CI): firmware host tests + MATLAB tests in Octave.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
make -C "$here/firmware/tests"
cd "$here/matlab/tests"
octave-cli --eval "set(0,'DefaultFigureVisible','off'); run_all_tests"
