# Repository URL — the one slot to fill before submission

**Status: TODO. No URL has been invented here.** Every occurrence of
`<URL>` and `<DOI-or-URL>` below is a placeholder that the corresponding
author must replace with the final public repository URL and the archival
DOI (or archived URL) once they exist.

This file is the single place that records the two edits required outside the
package. The package itself contains no repository URL; `CITATION.cff` carries
the same slot as `repository-code: "TODO(URL)"`.

## Place 1 — the manuscript's data-availability sentence

The manuscript currently says the materials "are provided as supplementary
material". Replace that sentence with exactly:

> The simulation code, the archived per-run results and the scripts that
> regenerate all tables and figures are available at `<URL>` (archived at
> `<DOI-or-URL>`).

LaTeX source (if the manuscript uses `\url`):

```latex
The simulation code, the archived per-run results and the scripts that
regenerate all tables and figures are available at \url{<URL>}
(archived at \url{<DOI-or-URL>}).
```

If the journal requires a separate "Code availability" and "Data
availability" statement, split the same sentence in two and keep both URLs
identical:

> **Code availability.** The simulation code and the scripts that regenerate
> all tables and figures are available at `<URL>` (archived at
> `<DOI-or-URL>`).
>
> **Data availability.** The archived per-run results are available at
> `<URL>` (archived at `<DOI-or-URL>`).

## Place 2 — the submission system's repository / data-availability field

In the submission system, open the field labelled **"Data availability"**,
**"Code availability"**, **"Repository URL"**, or **"Supplementary material
URL"** (the label differs by journal) and paste exactly this block:

```text
Repository: <URL>
Archived copy: <DOI-or-URL>
```

If the form accepts a single URL only, paste `<URL>` in the URL field and put
the full sentence from Place 1 in the accompanying data-availability text
box. Do not also upload the package as "supplementary material" unless the
journal explicitly requires a file upload; the sentence must point to the
repository instead.

## Also update — `CITATION.cff` (package metadata, not a third manuscript place)

The two places that matter for the submission are above. For completeness,
`CITATION.cff` at the package root also carries the same slot:

```yaml
repository-code: "TODO(URL)"
```

Replace `TODO(URL)` with the same `<URL>` used in Places 1 and 2, then update
the checksum manifests (§ below).

## BibTeX entry skeleton for `refs.bib`

Add this entry to `refs.bib` and cite it from the data-availability sentence
if the journal's style asks for a reference. Replace `<URL>` and
`<DOI-or-URL>`; if the archive is a Zenodo deposit, `<DOI-or-URL>` is normally
`https://doi.org/<DOI>`.

```bibtex
@dataset{wang2026ttc_repro,
  author       = {Wang, Shuaihu and Zhang, Gaowei and Xu, Zehao and Huang, Yun},
  title        = {{Animal contact-risk prediction and action-conditioned rollout
                   for multi-robot shepherding}: reproducibility package (v2)},
  year         = {2026},
  publisher    = {Zenodo},
  version      = {v2},
  doi          = {<DOI>},
  url          = {<URL>},
  note         = {Code: MIT License; data and results: CC BY 4.0}
}
```

If the archive is not on Zenodo, replace `publisher` and `doi` with the
actual archive (for example an institutional repository) and keep `url`
pointing at `<URL>`.

## Checklist before the package is published

- [ ] `<URL>` is the final public repository URL (no trailing slash surprises;
      use the canonical URL that resolves without login).
- [ ] `<DOI-or-URL>` is the archival identifier of the exact uploaded version.
- [ ] Place 1 sentence replaced in the manuscript; "supplementary material"
      no longer appears in the data-availability statement.
- [ ] Place 2 field filled in the submission system with the same two values.
- [ ] `CITATION.cff` `repository-code` updated from `TODO(URL)`.
- [ ] `python verify_package.py` still reports `errors = 0` after the edit
      (the manifest must be regenerated if `CITATION.cff` changes size).
- [ ] No `URL` placeholder remains anywhere: search the tree for `<URL>` and
      `TODO(URL)` after the edit.
