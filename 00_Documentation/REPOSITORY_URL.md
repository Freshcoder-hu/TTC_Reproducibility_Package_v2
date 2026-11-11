# Repository URL — the two slots, filled

**Status: FILLED (2026-11-10).** The canonical public repository is

> **https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2**

No placeholder token remains in this package. No archival
DOI has been assigned, so the repository URL is the identifier everywhere and
the `[dataset]` reference carries `url` but not `doi`.

## Slot 1 — the manuscript's data-availability sentence (FILLED)

The manuscript now ends with exactly one data-availability sentence, and the
repository is cited from it:

> **Data availability.** The simulation code, the archived per-run results and
> the scripts that regenerate all tables and figures are available at
> https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2 (Wang et al.,
> 2026a).

LaTeX source in `02_论文源文件/英文_LaTeX/cas/main.tex`:

```latex
\section*{Data availability}\label{sec:data}
{\raggedright
The simulation code, the archived per-run results and the scripts that
regenerate all tables and figures are available at
{\scriptsize\mbox{\url{https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2}}}
\mbox{\citep{wang2026ttc_repro}}.\par}
```

The URL is set in a `\mbox` so that it is never broken by line wrapping or
hyphenation; it is checked in the built PDF. "Provided as supplementary
material" no longer appears anywhere in the manuscript.

## Slot 2 — package metadata `CITATION.cff` (FILLED)

```yaml
repository-code: "https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2"
```

The four authors (Shuaihu Wang, Gaowei Zhang, Zehao Xu, Yun Huang) and the year
2026 are unchanged.

## Submission system — the field to paste (prepared, author action)

In the field labelled **Data availability**, **Code availability**,
**Repository URL** or **Supplementary material URL**, paste:

```text
Repository: https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2
```

The same sentence as Slot 1 goes in the accompanying text box. The repository
is public, so the code and results are not attached as supplementary material;
the Supplementary Material slot carries only the 14-page supplement.

## `[dataset]` reference added to `refs.bib`

`02_论文源文件/英文_LaTeX/cas/refs.bib` now ends with the activated skeleton,
cited from the data-availability sentence. The list is **41 entries** (40
journal articles with a DOI + this one dataset entry):

```bibtex
@misc{wang2026ttc_repro,
  author       = {Wang, Shuaihu and Zhang, Gaowei and Xu, Zehao and Huang, Yun},
  title        = {{[dataset] Animal contact-risk prediction and action-conditioned
                   rollout for multi-robot shepherding: reproducibility package (v2)}},
  year         = {2026},
  howpublished = {GitHub repository},
  url          = {https://github.com/Freshcoder-hu/TTC_Reproducibility_Package_v2},
  note         = {{Code: MIT License; data and results: CC BY 4.0. Accessed 10 November 2026}}
}
```

## Checklist — all satisfied on 2026-11-10

- [x] The manuscript URL is the final public repository URL, with no trailing
      slash.
- [x] Slot 1 replaced in the manuscript; "provided as supplementary material"
      is gone from the data-availability statement.
- [x] `CITATION.cff` `repository-code` carries the repository URL.
- [x] `refs.bib` carries the `[dataset]` entry and the manuscript cites it.
- [x] Reference count updated from 40 to 41 wherever the current paper is
      described (manuscript source comment, submission README, package READMEs).
- [x] Integrity manifests regenerated after the edit; `verify_package.py`
      reports `errors = 0`.
- [x] No placeholder token remains in the package tree.
