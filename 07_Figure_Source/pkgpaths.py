"""Path resolution for the figure scripts, mapped onto this repository.

Provenance
----------
These six scripts are the submission package's ``06_图表源码/``
(``fig_body.py``, ``fig_si.py``, ``fig_loss.py``,
``make_graphical_abstract.py``, ``pkgpaths.py``, ``v9style.py``).  In the
submission package they resolved their inputs against a different tree:

===========================================  ==================================
submission-package location                  this repository
===========================================  ==================================
``03_图表/正文图/`` (output)                  ``07_Figure_Source/build/body/``
``03_图表/补充图/`` (output)                  ``07_Figure_Source/build/si/``
``03_图表/`` (graphical abstract output)      ``07_Figure_Source/build/``
``04_数据/三弯道_240/``                       ``05_Curved_Path_240/``
``04_数据/机器人损毁_1080/``                  ``06_Robot_Loss_1080/``
``04_数据/百秒规模_未随包发布/``              **not shipped anywhere** (see P100)
nested archive, V2 tables                     ``01_Main_Experiment/paper/tables/``
nested archive, stage-3 observability         ``01_Main_Experiment/results/``
nested archive, robustness ``analysis/``      ``02_Perception_Robustness/analysis/``
nested archive, online ``analysis/``          ``03_Online_Warning_and_Timing/analysis/``
===========================================  ==================================

The submission package kept the V2 result tables inside a nested archive
(``TTC_Shepherding_Complete_Reproducibility_Package.zip``) and extracted it on
first use.  This repository ships those tables directly, and every input file
the scripts read is **byte-identical** to the copy inside that archive
(SHA-256 checked one by one; the mapping and the check are in ``README.md``).
So no extraction step is needed here.

``P100`` is the one input that neither tree ships: it is the 100 s-scale
panel of the earlier *Word* manuscript, which is not part of the submitted
article.  ``fig_body.py --only 9`` needs it; every other figure does not.

Override the repository root with ``PKG`` and the output directory with
``OUT`` (or the scripts' ``--out`` / ``--out-dir`` options) if needed.
"""
from __future__ import annotations

import os
from pathlib import Path

#: Repository root -- the directory that contains ``01_Main_Experiment`` …
ROOT = Path(os.environ.get(
    "PKG", Path(__file__).resolve().parents[1]))

# ------------------------------------------------------------------ inputs
MAIN = ROOT / "01_Main_Experiment"
TABLES = MAIN / "paper" / "tables"
STAGE3 = MAIN / "results" / "results_v2_stage3"
PERCEPTION = ROOT / "02_Perception_Robustness" / "analysis"
ONLINE = ROOT / "03_Online_Warning_and_Timing" / "analysis" / "online_measurement"
CURVED = ROOT / "05_Curved_Path_240"
LOSS = ROOT / "06_Robot_Loss_1080"

#: The Word-era 100 s-scale panel: not shipped by this repository or by the
#: submission package.  Point ``P100`` at a local copy to build that panel.
P100 = Path(os.environ.get("P100", ROOT / "07_Figure_Source" / "P100_not_shipped"))

# ------------------------------------------------------------------ outputs
#: ``build/`` is covered by the repository ``.gitignore``, so a run never
#: dirties the shipped tree or the integrity manifests.
BUILD = ROOT / "07_Figure_Source" / "build"
OUT_BODY = Path(os.environ.get("OUT_BODY", BUILD / "body"))
OUT_SI = Path(os.environ.get("OUT_SI", BUILD / "si"))
OUT_GA = Path(os.environ.get("OUT_GA", BUILD))


def need(path) -> Path:
    """Return *path* if it exists, else exit with an actionable message."""
    p = Path(path)
    if not p.exists():
        raise SystemExit(
            "missing input: %s\n"
            "Run the scripts from the repository root (the folder that "
            "contains 01_Main_Experiment), or set PKG to it." % p)
    return p
