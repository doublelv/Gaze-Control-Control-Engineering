# Gaze-Control-Control-Engineering
This is the public repo for Fonty's Master System Design's Control Engineering Class - Group 1

## Project scaffold

Monorepo scaffold for MATLAB eye-gaze software and C firmware targeting ST NUCLEO-H753ZI. No executable application, buildable firmware, or CI is configured yet.

- docs/: requirements, architecture, control theory, hardware, and decisions.
- matlab/: modular source, tests, scripts, and configuration.
- firmware/: generated-project locations, vendor drivers, application modules, and tests.
- interface/: shared protocol contract and interoperability vectors.
- tools/: development utilities.
- .github/workflows/: future automation.

Start by reading AGENTS.md and module READMEs, recording requirements and approved decisions, then implementing small tasks with measurable acceptance criteria. Support offline MATLAB testing with saved or synthetic input.

MATLAB release/toolboxes, camera, toolchain versions, control-loop allocation, pins, electrical limits, transport, protocol, timing, and safety settings remain TBD. This repository includes an MIT license; see LICENSE. Generate the actual STM32 project only after hardware decisions. README files retain every directory in Git, so empty source files and .gitkeep files are unnecessary. Scaffold creation does not authorize commits or pushes.
