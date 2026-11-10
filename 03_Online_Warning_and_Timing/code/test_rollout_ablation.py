"""Guard for the ablation rung: RolloutBare.step must be RolloutProposed.step
with exactly one call replaced (`super().step` -> `Proposed.step`).

Run:  python -m unittest test_rollout_ablation
"""
import inspect
import re
import textwrap
import unittest

from proposed_rollout import RolloutProposed
from proposed_rollout_ablation import RolloutBare

_CALLS = (
    "nominal,tag=super().step(view,cfg)",
    "nominal,tag=Proposed.step(self,view,cfg)",
)


def _norm_body(cls):
    src = textwrap.dedent(inspect.getsource(cls.step))
    lines = src.splitlines()[1:]                 # drop the `def step(...)` line
    out = []
    for ln in lines:
        ln = re.sub(r"\s+", "", ln)              # whitespace-insensitive
        for c in _CALLS:
            ln = ln.replace(c, "CALL")
        out.append(ln)
    return out


class TestAblationComposition(unittest.TestCase):
    def test_body_is_identical_apart_from_nominal_call(self):
        ref = _norm_body(RolloutProposed)
        bare = _norm_body(RolloutBare)
        self.assertEqual(len(ref), len(bare), "step bodies differ in length")
        for i, (a, b) in enumerate(zip(ref, bare)):
            self.assertEqual(a, b, f"line {i} differs:\n  full : {a}\n  bare : {b}")

    def test_bare_uses_proposed_nominal(self):
        src = textwrap.dedent(inspect.getsource(RolloutBare.step))
        self.assertIn("Proposed.step(self, view, cfg)", src)
        self.assertNotIn("super().step", src)

    def test_inherits_planner_and_name(self):
        self.assertTrue(issubclass(RolloutBare, RolloutProposed))
        self.assertEqual(RolloutBare._predict, RolloutProposed._predict)
        self.assertNotEqual(RolloutBare.name, RolloutProposed.name)


if __name__ == "__main__":
    unittest.main()
