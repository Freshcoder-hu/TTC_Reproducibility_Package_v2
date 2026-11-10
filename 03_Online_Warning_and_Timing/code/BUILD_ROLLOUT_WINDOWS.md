# BUILD_ROLLOUT_WINDOWS.md

Short pointer: the Windows build of the rollout kernel is documented in
[`WINDOWS_PORT.md`](WINDOWS_PORT.md) §2.

Quick reference (Developer Command Prompt for VS, from this directory):

```
cl /LD /O2 /EHsc /D_USE_MATH_DEFINES rollout.cpp /link /EXPORT:evaluate_rollouts
```

produces `_rollout.dll` next to `rollout.cpp`. `proposed_rollout.py` and
`run_improvement.py` look for exactly `_rollout.dll` on Windows and abort with
`build _rollout.dll first` if it is missing. There is **no** `g++` step on
Windows: the loader patch in `proposed_rollout.py` replaces the Linux compile
step entirely.
