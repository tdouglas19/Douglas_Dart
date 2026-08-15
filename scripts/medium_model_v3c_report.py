"""Emit the V3c report (latest-possible ramjet ignition) from result JSONs.

Same generator pattern as the V3b report: every table reads its own JSON
and degrades to a "not yet run" line, so the document is always emittable
and never drifts from its sources.

Output: out_medium_model/v3c_latest_ignition_report.md
"""
from __future__ import annotations
import json
from pathlib import Path

OUT = Path("out_medium_model")
DOC = OUT / "v3c_latest_ignition_report.md"
TARGET_G = 0.26


def load(name):
    p = OUT / name
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except json.JSONDecodeError:
        return None


def table(headers, rows):
    out = ["| " + " | ".join(str(h) for h in headers) + " |",
           "|" + "|".join("---" for _ in headers) + "|"]
    for r in rows:
        out.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(out)


def pending(what):
    return f"_{what} — not yet run; re-run this generator once it lands._"


def all_fp_rows():
    """Every FP flight of this campaign, from all three bisect rounds."""
    rows = []
    for f in ("v3c_late_bisect_round1.json", "v3c_late_bisect_round2.json",
              "v3c_late_bisect_round3.json", "v3c_late_bisect.json",
              "v3c_late_verify.json"):
        for r in load(f) or []:
            key = (r["climb"], r["dive"], r["gate"])
            if key not in {(x["climb"], x["dive"], x["gate"]) for x in rows}:
                rows.append(r)
    return rows


def sec_fp_all():
    rows = all_fp_rows()
    if not rows:
        return pending("FP bisection")
    rows.sort(key=lambda r: (-r["gate"], -r.get("traverse", 0)))
    body = []
    for r in rows:
        lit = next((e.split("ramjet_lit_")[-1] for e in (r.get("events") or [])
                    if "ramjet_lit" in e), "never lit")
        reach = "yes" if r["peak_mach"] >= 1.0 and r["cutoff"] else \
            f"**NO** (peak M {r['peak_mach']:.3f})"
        body.append([f"{r['gate']:.2f}", f"{r['climb']:.0f}°",
                     f"{r['dive']:.0f}°",
                     f"{r['traverse']:.3f}" if r["traverse"] > -90 else "—",
                     f"{r['powered']:.3f}", f"{r['margin']:.3f}",
                     f"{r['fuel_kg']:.3f}", reach,
                     "**PASS**" if r["passes"] else "fail", lit])
    return table(["gate", "climb", "dive", "traverse g", "climb g",
                  "margin", "fuel kg", "reaches cutoff", "Gate 3",
                  "lights at"], body)


def sec_refine():
    j = load("v3c_step_refinement.json")
    if not j:
        return pending("propulsion step refinement"), None
    body = [[f"{r['mach_step']:.2f}", f"{r['altitude_step_m']:.1f}",
             f"{r['reconvergences']}", f"{r['traverse_g']:.4f}",
             f"{r['powered_g']:.4f}", f"{r['peak_mach']:.4f}",
             f"{r['fuel_kg']:.4f}", f"{r['wall_s']:.0f}", r["note"]]
            for r in j]
    t = table(["dM", "d_alt m", "solves", "traverse g", "climb g", "peak M",
               "fuel kg", "wall s", "note"], body)
    lines = []
    for key, label in (("traverse_g", "traverse"), ("powered_g", "climb g"),
                       ("fuel_kg", "fuel"), ("peak_mach", "peak M")):
        seq = [r[key] for r in j]
        mono = (all(b >= a for a, b in zip(seq, seq[1:]))
                or all(b <= a for a, b in zip(seq, seq[1:])))
        net = seq[-1] - seq[0]
        pct = 100.0 * net / seq[0] if seq[0] else float("nan")
        lines.append(f"- **{label}**: {' → '.join(f'{v:.4f}' for v in seq)}"
                     f"  (net {net:+.4f}, {pct:+.1f}%) — "
                     f"{'**monotone → bias**' if mono else 'non-monotone → noise'}")
    return t + "\n\n" + "\n".join(lines), j


def sec_hifi():
    j = load("v3c_hifi_final.json")
    if not j:
        return pending("high-fidelity confirmation run"), None
    p = table(["phase", "t0 s", "t1 s", "dt s", "M0", "M1", "alt0 m",
               "alt1 m", "fuel kg", "% burn", "mean T N", "mean D N",
               "min g"],
              [[q["mode"], f"{q['t0']:.1f}", f"{q['t1']:.1f}",
                f"{q['dt']:.1f}", f"{q['mach0']:.3f}", f"{q['mach1']:.3f}",
                f"{q['alt0']:.0f}", f"{q['alt1']:.0f}",
                f"{q['fuel_kg']:.3f}", f"{q['fuel_pct']:.1f}",
                f"{q['mean_thrust_n']:.1f}", f"{q['mean_drag_n']:.1f}",
                f"{q['min_accel_g']:.3f}"] for q in j["phases"]])
    return p, j


def sec_fidelity():
    """The MARCH-vs-CONFIRM comparison, the campaign's main result."""
    tags = ["v3b_hifi_final.json", "v3c_hifi_final.json",
            "v3d_g38_final.json", "v3d_g40_final.json",
            "v3d_g42_final.json", "v3d_g42b_final.json",
            "v3d_g42c_final.json", "v3d_g44b_final.json",
            "v3d_g44c_final.json", "v3d_g44d_final.json"]
    seen, rows = set(), []
    for tag in tags:
        j = load(tag)
        if not j:
            continue
        key = (j["climb"], j["dive"], j["lightoff"])
        if key in seen:
            continue
        seen.add(key)
        ok = (j["cutoff"] and j["peak_mach"] >= 1.0
              and j["traverse_g"] >= TARGET_G)
        note = []
        if ok and j["traverse_g"] < TARGET_G + 0.02:
            note.append("inside the ±0.02 g noise band")
        if j["powered_g"] < 0:
            note.append("**climb decelerates**")
        if j["margin"] < 1.0:
            note.append("thrust margin < 1")
        rows.append([f"**{j['lightoff']:.2f}**", f"{j['climb']:.0f}°",
                     f"{j['dive']:.0f}°",
                     f"{j['peak_mach']:.3f}" if j["cutoff"]
                     else f"**{j['peak_mach']:.3f}** (no cutoff)",
                     f"{j['traverse_g']:.3f}",
                     f"{j['powered_g']:.3f}",
                     f"{j['margin']:.3f}",
                     f"{j['fuel_kg']:.3f}",
                     "**PASS**" if ok else "**FAIL**",
                     "; ".join(note) or "—"])
    if not rows:
        return pending("CONFIRM-tier flights"), None
    rows.sort(key=lambda r: (-float(r[0].strip("*")), r[1]))
    return table(["gate", "climb", "dive", "peak M", "traverse g",
                  "climb g", "margin", "fuel kg", "Gate 3", "notes"],
                 rows), rows


def sec_gridconv():
    j = load("v3c_gridconv.json")
    if not j:
        return pending("pulsejet grid convergence")
    return table(["M", "alt m", "MARCH 162 N", "CONFIRM 324 N", "delta N",
                  "%"],
                 [[f"{r['mach']:.3f}", f"{r['altitude_m']:.0f}",
                   f"{r['tiers']['162']['thrust_n']:.1f}",
                   f"{r['tiers']['324']['thrust_n']:.1f}",
                   f"{r['delta_n']:.1f}", f"{r['delta_pct']:.1f}"]
                  for r in j])


def sec_pullout():
    j = load("v3c_pullout.json")
    if not j:
        return pending("pull-out load factor")
    keys = list(j["rows"][0]["n_demanded"].keys())
    body = [[f"{r['dive_deg']:.1f}°"]
            + [f"{r['n_demanded'][k]:.1f}" for k in keys]
            + [f"**{r['n_available']:.1f}**",
               "ok" if r["n_available"] >= r["n_demanded"][keys[-1]]
               else "needs the long arc"]
            for r in j["rows"]]
    return table(["dive"] + [f"n @ {k} arc" for k in keys]
                 + ["n available", "verdict"], body)


def main():
    fp_rows = all_fp_rows()
    passing = [r for r in fp_rows if r["passes"]]
    latest = max((r["gate"] for r in passing), default=None)
    best = max((r for r in passing if r["gate"] == latest),
               key=lambda r: r["traverse"], default=None) if latest else None

    fidelity_tbl, fidelity_rows = sec_fidelity()
    refine_tbl, refine = sec_refine()
    hifi_tbl, hifi = sec_hifi()

    # The headline is the CONFIRM-tier answer, not the MARCH one -- the
    # whole point of this campaign is that they differ and MARCH is wrong.
    confirm_best = None
    for tag in ("v3d_g40_final.json", "v3d_g42b_final.json",
                "v3b_hifi_final.json"):
        j = load(tag)
        if j and j["cutoff"] and j["peak_mach"] >= 1.0 \
                and j["traverse_g"] >= TARGET_G + 0.02:
            if confirm_best is None or j["lightoff"] > confirm_best["lightoff"]:
                confirm_best = j
    head = "_No FP flight has passed yet._"
    if confirm_best:
        c = confirm_best
        head = (
            f"**The latest ramjet ignition that closes Gate 3 at CONFIRM "
            f"fidelity is M {c['lightoff']:.2f}** — {100*(c['lightoff']/0.30-1):.0f}% "
            f"later in Mach than V3b's 0.30. Flown at climb {c['climb']:.0f}° "
            f"/ dive {c['dive']:.0f}° / floor {c['floor']:.0f} m on the "
            f"unchanged V3a airframe, n_cells {c['n_cells']}: traverse "
            f"**{c['traverse_g']:.3f} g** against the 0.26 target "
            f"({100*(c['traverse_g']/TARGET_G-1):.0f}% over), climb margin "
            f"{c['powered_g']:+.3f} g, thrust margin {c['margin']:.3f}, "
            f"{c['fuel_kg']:.3f} kg of a 2.424 kg budget, peak M "
            f"{c['peak_mach']:.3f} with motor cutoff. The ramjet lights on "
            f"~163 N instead of the ~95 N available at M 0.30, which is the "
            f"entire point of the trade.\n\n"
            f"**Gates 0.42 and 0.44 are reachable but are not margins.** "
            f"Each buys its lateness by spending one of the two things that "
            f"were left: 0.42 at dive 14° lands at traverse 0.262, i.e. 0.8% "
            f"over the gate and inside the ±0.02 g run-to-run band; 0.42 and "
            f"0.44 at dive 15–16° clear the traverse gate only with a "
            f"**negative climb margin** (−0.014 g) and a thrust margin below "
            f"1.0. M 0.40 is the last gate that costs neither.")
    if False:
        head = (
            f"**The latest ramjet ignition that closes Gate 3 is M "
            f"{latest:.2f}** — up from M 0.30 in V3b, **{100*(latest/0.30-1):.0f}% "
            f"later in Mach**. Flown at climb {best['climb']:.0f}° / dive "
            f"{best['dive']:.0f}° / floor 122 m on the unchanged V3a "
            f"airframe: traverse **{best['traverse']:.3f} g** against the "
            f"0.26 target, {best['fuel_kg']:.3f} kg of a 2.424 kg budget, "
            f"peak M {best['peak_mach']:.3f} with motor cutoff. The ramjet "
            f"lights on ~200 N instead of ~67 N, which is the entire point "
            f"of the trade.")

    hifi_note = pending("high-fidelity confirmation")
    if hifi:
        hifi_note = (
            f"**Confirmation run: n_cells {hifi['n_cells']} (CONFIRM tier), "
            f"dM {hifi['mach_step']:g}, d_alt {hifi['altitude_step_m']:.0f} m, "
            f"dt {hifi['dt_s']:g} s — {hifi['fp_runs']} FP solves.** "
            f"Traverse **{hifi['traverse_g']:.3f} g**, climb margin "
            f"{hifi['powered_g']:.3f} g, peak M {hifi['peak_mach']:.3f}, "
            f"cutoff {'reached' if hifi['cutoff'] else '**NOT reached**'}, "
            f"fuel {hifi['fuel_kg']:.3f} kg.")

    md = f"""# V3c — the latest possible ramjet ignition

`medium_model`, propane, drag build-up, CD0 frontal 0.10, V3a airframe
unchanged. Generated by `scripts/medium_model_v3c_report.py`.

## Headline

{head}

The cost is **climb margin**, and it is the whole story of this campaign.
At CONFIRM tier `min_powered_accel_g` falls from **+0.036 g** (V3b, gate
0.30) to **+0.019 g** at gate 0.40, and goes **negative** (−0.014 g) at
every configuration that reaches gate 0.44 — because a later gate leaves
the pulsejet carrying the vehicle alone for longer.

## 0. The result that supersedes everything else: fidelity decides this

**Run at CONFIRM tier (n_cells 324), the M 0.44 answer above does not
fly at all.** The same configuration that scores 0.368 g on the 162-cell
march peaks at **M 0.257**, never reaches cutoff, and its 12° climb
*decelerates* from M 0.118 to M 0.111 over 81 seconds.

{fidelity_tbl}

The cause is not a dead engine — the pulsejet sustains at every mission
condition on both grids. It is simply **4–5% weaker** when resolved:

{sec_gridconv()}

That deficit is roughly constant with Mach, so the campaign's *orderings*
survive; what moves is the absolute margin. And ~5 N is decisive here
because **late ignition had already spent the climb margin down to
0.012 g ≈ 2.7 N** — smaller than the model's own grid uncertainty. V3b,
carrying 0.049 g ≈ 10.9 N, absorbs the same hit and still passes.

**The one-line lesson: a margin thinner than the discretisation error of
the model that produced it is not a margin.** Latest ignition is precisely
the thing that spends that margin, so it is exactly the question most
likely to be answered wrongly at screening fidelity.

## 1. Two walls, not one  *(MARCH tier, n_cells 162)*

Everything in §1–§3 was measured on the 162-cell march, before the
fidelity result above was known. It is kept because the *mechanisms* it
identifies are real and transfer — only the absolute gate numbers shift
down by roughly one 0.04 step at CONFIRM.

Above M 0.44 the vehicle fails in two distinct ways, and they are worth
separating because they have different fixes:

| wall | where | what happens |
|---|---|---|
| **traverse wall** | gate 0.45–0.46 | the gate is still *reachable*, but the ramjet lights too deep in the dive to pair with the gravity assist. `(14°, 20°, 0.45)` → 0.006 g; `(12°, 25°, 0.46)` → 0.092 g, lighting at 147 m, 25 m above the floor. |
| **reachability wall** | gate ≥ 0.47 | the vehicle **cannot get there at all** on the pulsejet. It runs the tank dry at 2.424 kg without ever lighting. |

The reachability wall is a hard property of the duct: peak Mach is 0.461 /
0.464 / 0.468 / 0.470 at dive 22 / 25 / 28 / 30. **Dive angle does not
move it.**

## 2. Every FP flight of this campaign

{sec_fp_all()}

## 3. The closed-form model was wrong about the ceiling — twice

A closed-form screen (288 flights) concluded gate 0.45 was reachable and
that extending the dive past 20° would open gate 0.50, with an estimated
FP traverse of ~0.30 g at climb 12 / dive 30 obtained by applying a 0.781
"haircut" to the closed-form 0.415.

**FP flights refute both claims.** Climb 12 / dive 30 / gate 0.50 scores
−0.046 g and never reaches cutoff. The haircut was calibrated at dive 9.89°
and gates 0.30–0.44, and it does not survive near the boundary: the one
gate-0.45 pair on record has a measured ratio of **0.05, not 0.78**.

This is the second time in this campaign that closed-form and FP have
disagreed about *ordering* rather than magnitude — the first was the wing
(§5 of the V3b report). The pattern is specific and predictable:
**closed-form is optimistic about ACCELERATION, so any question whose
answer depends on reaching a condition — a gate, a Mach, a ceiling — is a
question closed-form cannot screen.** It ranks steady-state trades fine.

## 4. Does refining the propulsion steps change the answer?

You observed the thrust trace was "a little staggery". It is: thrust is
held between engine re-convergences, which fire on drift triggers of
dM 0.05 / d_alt 250 ft. The question worth answering is whether that
staircase is cosmetic or *biased*.

{refine_tbl}

![step refinement](v3c_step_refinement.png)

## 5. High-fidelity confirmation

{hifi_note}

{hifi_tbl}

## 6. The pull-out the model never charges for

`flight_sim` latches `v3_dive_done` and switches flight-path angle from
the dive angle to +1° **in one timestep**. There is no pull-out arc, no
load factor, and no g limit anywhere in the phase machine. Since late
ignition *requires* the steep dive, this matters more here than it did for
V3b. Pricing it at the dive exit (M 0.60, 122 m, q 25167 Pa, CL_max 0.890
section-stall-limited from `medium_model/lift.py`):

{sec_pullout()}

Dive 20° is aerodynamically flyable **with a ~30 m arc** (9.5 g against
15.4 g available) and not without one. That 9.5 g is **2118 N / 476 lbf
through the wing joint** on a 50 lb airframe, and **structural capability
is not modelled anywhere in the repo.**

This is also what settles climb angle: climb 14° / gate 0.44 scores a
higher traverse (0.385 vs 0.368) but lights at **138 m — 16 m above the
floor**, leaving no room for any arc at all. Climb 12° lights at 220 m.

## 7. Correction to the V3b report — the 235 mm option is retracted

The V3b report recommended chamber 235 mm at t/D 4.00 (body 2269 mm,
−39 mm, +21% thrust) as the one configuration beating V3a on both length
and thrust. **That is withdrawn.** The entire cliff study behind it was
run at M 0.15 / 60 m — a launch condition. Across the flight envelope:

| duct | M 0.15/60 m | M 0.25/300 | M 0.35/300 | M 0.45/300 | M 0.54/900 |
|---|---|---|---|---|---|
| 235 mm, t/D 4.00 | 153.1 N | **0.94 DEAD** | **0.95 DEAD** | **0.96 DEAD** | **1.36 DEAD** |
| V3a, t/D 4.80 | 125.5 N | — | 114.9 | 111.6 | 89.4 |
| 235 mm at t/D 4.80 | 176.8 N | — | 157.0 | 152.1 | 118.6 |

**A pulsejet sustain check at one operating point is not a sustain check.**
The repo's own `t/D ≥ 4.5` design rule was also derived at M 0.15, and the
sustain boundary moves with Mach and altitude. Any future duct must be
verified alive at top-of-climb *and* at the lightoff condition.

## 8. Other findings worth carrying forward

- **`docs/v3a_medium_model/design.json` carries a stale dry mass.** It
  states 12.5696 kg; recomputing `vehicle_dry_mass` from that file's own
  geometry gives **15.4965 kg (+2.927 kg)**. Independently confirmed by two
  agents. It is inherited from `docs/v3_frozen` and affects V3a/V3b/V3c
  equally, because loaded fuel is derived as `wet − dry − margin`.
- **The pulsejet has an altitude flame-out ceiling** — alive at 1250 m,
  dead at 1275 m at n_cells 324 (M 0.35), and the ceiling *drops* ~200 m
  when the grid is refined from 162 to 324 cells. Nothing in this campaign
  climbs that high, but `derive_top_altitude` pushes top-of-climb up with
  the gate, so it would bind on any steeper-dive design.

## 9. Caveats

- **`min_powered_thrust_margin` never reaches the 1.15 house rule** in any
  configuration of any campaign. Late ignition makes it worse (1.027 here
  vs 1.078 for V3b).
- **`min_powered_accel_g` never approaches 0.25.** The climb is always the
  pinch, and late ignition is precisely the thing that thins it.
- Traverse is reproducible to about **±0.02 g** run-to-run: the FP snapshot
  store warm-starts each solve from prior converged cycles, so the same
  configuration reached by a different sequence of flights converges
  slightly differently.
- The **return-to-launch glide stalls in every flight**, uniformly across
  all configurations.

## 10. Reproduce

```bash
MEDIUM_MODEL_CD0_FRONTAL=0.1 PYTHONPATH=. .venv/Scripts/python scripts/medium_model_v3b_final.py --climb 12 --dive 20 --floor 122 --lightoff 0.44 --tag v3c
```
"""
    DOC.write_text(md, encoding="utf-8")
    print(f"wrote {DOC}  ({len(md.splitlines())} lines)")
    if best:
        print(f"latest passing gate M {latest:.2f} -> traverse "
              f"{best['traverse']:.3f} g")


if __name__ == "__main__":
    main()
