import csv
p = "results_v2_stage4/geometry/s3_spacing_sweep.csv"
for r in csv.DictReader(open(p, encoding="utf-8-sig")):
    print(f"e2={r['e2']:>5} s{r['seed']} "
          f"det=({float(r['detect_1']):6.2f},{float(r['detect_2']):6.2f},"
          f"{float(r['detect_3']):6.2f}) "
          f"onset=({float(r['onset_1']):6.2f},{float(r['onset_2']):6.2f},"
          f"{float(r['onset_3']):6.2f}) rec={r['recover_windows']}")
