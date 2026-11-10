"""Original runner with explicit new method and common physical corrections.

Example:
  python run_revised_experiments.py --all --seeds 30 --seed-offset 200 \
      --algs Ours Ours_rollout B1 B2 --jobs 8 --out results_revised_full

Requires Linux/WSL, Python, NumPy, SciPy and g++ (C++17).
The original files and ALGS['Ours'] are not overwritten.
"""
import corrections
corrections.install(environment=True,contact=True)
from proposed_rollout import RolloutProposed
from run_experiments import ALGS,main

ALGS['Ours_rollout']=lambda **kw:RolloutProposed(**{
    k:v for k,v in kw.items() if k in ('divert_shift','pre_shift_ratio','t_pre','t_divert')})

if __name__=='__main__':
    # Ensure workers load a complete native library, including under spawn.
    RolloutProposed()
    main()
