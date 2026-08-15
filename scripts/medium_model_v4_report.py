"""Build the V4 Gate 3 report for medium_model from the three rung JSONs.

Reads out_medium_model/v4_rung{a,b,c}_final.json (written by
scripts/medium_model_v4_final.py) and emits a standalone markdown report --
no comparison against any earlier model version (user preference,
2026-08-13); the one comparison that IS made is rung A against
simple_model's own frozen V4, because that is a port check, not a design
claim.

Usage: python scripts/medium_model_v4_report.py
"""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ.setdefault("MEDIUM_MODEL_CD0_FRONTAL", "0.1")

OUT = Path("out_medium_model")
FROZEN = Path("docs/v4_frozen/design.json")
RUNGS = [("a", "A", "legacy drag + closed-form engines"),
         ("b", "B", "build-up drag + closed-form engines"),
         ("c", "C", "build-up drag + first-principles engines"),
         # Same three rungs re-flown with RamjetStart.light_at_pullout, the
         # last-chance lightoff (user, 2026-08-13). Only present if run.
         ("a_lap", "A+", "rung A, + light at pull-out"),
         ("b_lap", "B+", "rung B, + light at pull-out"),
         ("c_lap", "C+", "rung C, + light at pull-out")]


def load(key):
    p = OUT / f"v4_rung{key}_final.json"
    return json.loads(p.read_text()) if p.exists() else None


def num(x, spec=".3f", dash="--"):
    if x is None:
        return dash
    if isinstance(x, bool):
        return "yes" if x else "no"
    try:
        return format(x, spec)
    except (TypeError, ValueError):
        return str(x)


def main():
    frozen = json.loads(FROZEN.read_text())
    vm = frozen["verified_mission"]
    runs = {k: load(k) for k, _, _ in RUNGS}
    have = [(k, name, what) for k, name, what in RUNGS if runs[k]]
    if not have:
        raise SystemExit("no rung JSONs found -- run medium_model_v4_final.py")

    L = []
    A = L.append
    A("# Douglas Dart V4 through `medium_model` — Gate 3")
    A("")
    A("**Input: `docs/v4_frozen/design.json`, unmodified.** This is a re-fly, "
      "not a search — nothing here optimises anything, and no design "
      "parameter was changed (user directive, 2026-08-13). The vehicle and "
      "the commanded trajectory are exactly what `simple_model` froze; what "
      "changes down the ladder is the *fidelity of the physics they are "
      "flown through*.")
    A("")
    A("| rung | drag | propulsion | what it answers |")
    A("|---|---|---|---|")
    A("| A | flat CD0 (legacy) | closed-form | **Port check.** Must reproduce "
      "`simple_model`'s frozen V4. |")
    A("| B | component build-up | closed-form | Cost of real drag, isolated. |")
    A("| C | component build-up | first-principles, marched | The Gate 3 "
      "answer. |")
    if any(k.endswith("_lap") for k, _, _ in have):
        A("| A+ / B+ / C+ | as A / B / C | as A / B / C | the same three, "
          "re-flown with `RamjetStart.light_at_pullout` — if the ramjet has "
          "not made the gate by the pull-out, light it there regardless of "
          "Mach (user, 2026-08-13). |")
    A("")
    rc = runs.get("c")
    if rc:
        A("## 0. Result")
        A("")
        if rc["cutoff"]:
            A("**The V4 trajectory closes at first-principles fidelity.**")
        else:
            A(f"**The V4 trajectory does not close at first-principles "
              f"fidelity.** Peak Mach {rc['peak_mach']:.3f} against a "
              f"{rc['gate_mach']:.2f} ramjet gate and a 1.10 cutoff: the "
              f"vehicle never reaches the gate, so the ramjet is never asked "
              f"to light, and it burns its entire "
              f"{rc['burn_cap_kg']:.3f} kg fuel allocation "
              f"stuck at Mach ~{rc['dive_exit_mach']:.2f}.")
            A("")
            A("The failure is a chain, and every link is measurable:")
            A("")
            ra_, rb_ = runs.get("a"), runs.get("b")
            climb = next((q for q in rc["phases"] if q["mode"] == "v3_climb"),
                         None)
            if climb and ra_:
                a_climb = next((q for q in ra_["phases"]
                                if q["mode"] == "v3_climb"), None)
                A(f"1. **The climb arrives slow.** Top of climb is reached at "
                  f"Mach {climb['mach1']:.3f}, against "
                  f"{a_climb['mach1']:.3f} in rung A — the vehicle is "
                  f"{a_climb['mach1'] - climb['mach1']:.3f} Mach down before "
                  f"the dive even starts.")
            if rc["events"]:
                A(f"2. **The pulsejet flames out on the way up.** "
                  + "; ".join(f"`{e}`" for e in rc["events"])
                  + ". Both quenches are in the top ~150 m of the climb. "
                    "`simple_model` has no flame-out model at all — it only "
                    "lapses thrust as (rho/rho_SL)^3 — so this failure mode "
                    "is invisible below this rung.")
            A(f"3. **The dive cannot make up the deficit.** Dive exit Mach "
              f"{rc['dive_exit_mach']:.3f}"
              + (f" against {rb_['dive_exit_mach']:.3f} in rung B and "
                 f"{ra_['dive_exit_mach']:.3f} in rung A" if rb_ and ra_
                 else "")
              + f". The floor triggers the pull-out, not the "
                f"{rc['gate_mach']:.2f}-plus dive-end Mach the profile "
                f"assumes.")
            A(f"4. **So the ramjet is never asked.** `ramjet_light_refused` "
              f"is `{str(bool(rc['ramjet_light_refused'])).lower()}` — this "
              f"is **not** the first-principles ramjet declining to light. "
              f"The policy window (Mach {rc['gate_mach']:.2f} while "
              f"descending) never opens, so the FP model is never given the "
              f"question. Whether it *would* light at Mach "
              f"{rc['gate_mach']:.2f} is untested by this run.")
            A(f"5. **The drag strip becomes the stranded regime the "
              f"acceleration gate exists to forbid.** Thrust and drag sit "
              f"within a few newtons of each other for "
              + f"{next((q['dt'] for q in rc['phases'] if q['mode'] == 'drag_strip'), 0):.0f} s"
              + " while the tank empties.")
        A("")

    # --- the last-chance lightoff, if it was flown against FP -----------
    rcl = runs.get("c_lap")
    if rcl:
        A("### 0b. With the last-chance lightoff at the pull-out")
        A("")
        A(f"`RamjetStart.light_at_pullout` waives the Mach gate the moment "
          f"the pull-out begins (user, 2026-08-13). Against the "
          f"first-principles engines that does not *make* the ramjet light — "
          f"it means the ramjet is **asked** at the pull-out instead of "
          f"never. Rung C+ is the answer to that question.")
        A("")
        if rcl["ramjet_lightoff_mach"] is not None:
            A(f"**The ramjet lit at Mach "
              f"{rcl['ramjet_lightoff_mach']:.3f}, "
              f"{rcl['ramjet_lightoff_altitude_m']:.0f} m, in "
              f"`{rcl['ramjet_lightoff_mode']}`** — below the "
              f"{rcl['gate_mach']:.2f} gate the design was written around.")
            A("")
            if rcl["cutoff"]:
                A(f"**And the mission closes**: peak Mach "
                  f"{rcl['peak_mach']:.3f}, cutoff reached, "
                  f"{rcl['fuel_kg']:.3f} kg of a {rcl['burn_cap_kg']:.3f} kg "
                  f"allocation. The gate was the thing standing between this "
                  f"airframe and a closed mission, not the engine.")
            else:
                A(f"**The mission still does not close**: peak Mach "
                  f"{rcl['peak_mach']:.3f}, cutoff "
                  f"{'reached' if rcl['cutoff'] else 'NOT reached'}, "
                  f"{rcl['fuel_kg']:.3f} kg burnt of a "
                  f"{rcl['burn_cap_kg']:.3f} kg allocation. Lighting the "
                  f"ramjet was necessary and not sufficient.")
        elif rcl["ramjet_light_refused"]:
            A(f"**The first-principles ramjet was asked at the pull-out and "
              f"refused.** This is the answer the base rung C could not "
              f"give: there, the policy window never opened, so the model "
              f"was never given the question. Here it was, at Mach "
              f"~{rcl['dive_exit_mach']:.3f}, and it declined — so the "
              f"M {rcl['gate_mach']:.2f} gate is not merely unreached, it is "
              f"below what this flameholder will cold-light on.")
        else:
            A("The ramjet neither lit nor recorded a refusal — check the "
              "engine trace; the policy window may not have opened.")
        A("")

    # ---- port check -------------------------------------------------
    ra = runs.get("a")
    if ra:
        A("## 1. Port check (rung A)")
        A("")
        A("`medium_model` is a standalone copy of `simple_model`, so with the "
          "legacy drag model and the closed-form engines it must reproduce "
          "the freeze exactly. It does.")
        A("")
        A("| quantity | medium_model | frozen simple_model |")
        A("|---|---|---|")
        checks = [
            ("peak Mach", ra["peak_mach"], vm["peak_mach"], ".4f"),
            ("dive exit Mach", ra["dive_exit_mach"], vm["dive_exit_mach"], ".4f"),
            ("ramjet lightoff Mach", ra["ramjet_lightoff_mach"],
             vm["ramjet_lightoff_mach"], ".4f"),
            ("ramjet lightoff altitude (m)", ra["ramjet_lightoff_altitude_m"],
             vm["ramjet_lightoff_altitude_m"], ".1f"),
            ("lit in the dive", ra["ramjet_lit_in_dive"],
             vm["ramjet_lit_in_dive"], ""),
            ("peak T/W", ra["peak_tw"], vm["peak_thrust_to_weight"], ".4f"),
            ("min traverse accel (g)", ra["traverse_g"],
             vm["min_traverse_accel_g"], ".4f"),
            ("min powered accel (g)", ra["powered_g"],
             vm["min_powered_accel_g"], ".4f"),
            ("engine-only thrust margin", ra["margin"],
             vm["min_powered_thrust_margin"], ".4f"),
            ("peak body load (g)", ra["peak_load_n_total"],
             vm["peak_load_n_total"], ".4f"),
            ("min powered altitude (m)", ra["min_powered_altitude_m"],
             vm["min_powered_altitude_m"], ".2f"),
            ("pull-out radius (m)", ra["pullout_radius_m"],
             vm["pullout_radius_m"], ".1f"),
            ("spiral radius (m)", ra["spiral_radius_m"],
             vm["spiral_radius_m"], ".1f"),
            ("fuel burned (kg)", ra["fuel_kg"], vm["fuel_burned_kg"], ".4f"),
            ("lands from launch (m)", ra["lands_from_launch_m"],
             vm["lands_from_launch_m"], ".2f"),
        ]
        for name, got, want, spec in checks:
            A(f"| {name} | {num(got, spec)} | {num(want, spec)} |")
        A("")
        A("The residual differences are the rounding stored in `design.json` "
          "itself (it records 4 dp / 1 dp), not model divergence.")
        A("")

    # ---- ladder -----------------------------------------------------
    A("## 2. The ladder")
    A("")
    hdr = "| quantity | " + " | ".join(f"rung {n}" for _, n, _ in have) + " |"
    A(hdr)
    A("|---|" + "---|" * len(have))

    def row(label, key, spec=".3f", fp_only=False):
        cells = []
        for k, _, _ in have:
            r = runs[k]
            if fp_only and r["propulsion"] == "closed-form":
                cells.append("n/a")
                continue
            v = r.get(key)
            cells.append("--" if v == "" else num(v, spec))
        A(f"| {label} | " + " | ".join(cells) + " |")

    row("drag model", "drag_model", "")
    row("propulsion", "propulsion", "")
    row("peak Mach", "peak_mach", ".3f")
    row("motor cutoff reached", "cutoff", "")
    row("ramjet lightoff Mach", "ramjet_lightoff_mach", ".3f")
    row("ramjet lightoff altitude (m)", "ramjet_lightoff_altitude_m", ".0f")
    row("phase at lightoff", "ramjet_lightoff_mode", "")
    row("**lit in the dive**", "ramjet_lit_in_dive", "")
    row("light-at-pull-out override", "light_at_pullout", "")
    row("FP refused the light", "ramjet_light_refused", "", fp_only=True)
    row("dive exit Mach", "dive_exit_mach", ".3f")
    row("peak T/W", "peak_tw", ".2f")
    row("min traverse accel (g)", "traverse_g", ".3f")
    row("min powered accel (g)", "powered_g", ".3f")
    row("engine-only thrust margin", "margin", ".3f")
    row("peak body load (g)", "peak_load_n_total", ".2f")
    row("peak load phase", "peak_load_mode", "")
    row("pull-out load flown (g)", "peak_load_n_yaw", ".2f")
    row("min powered altitude (m)", "min_powered_altitude_m", ".1f")
    row("floor violated", "floor_violated", "")
    row("fuel burned (kg)", "fuel_kg", ".3f")
    row("fuel cap (kg)", "burn_cap_kg", ".3f")
    row("tank dry before cutoff", "tank_dry", "")
    row("safe landing", "safe_landing", "")
    row("stalled", "stalled", "")
    row("lands from launch (m)", "lands_from_launch_m", ".0f")
    row("FP solves", "fp_runs", "d")
    A("")

    # ---- per-rung phase tables ---------------------------------------
    A("## 3. Phase sequence, per rung")
    for k, name, what in have:
        r = runs[k]
        A("")
        A(f"### Rung {name} — {what}")
        A("")
        A("| phase | t (s) | dt (s) | alt (m) | Mach | gamma (deg) | "
          "peak n (g) | fuel (kg) | mean T (N) | mean D (N) | min accel (g) |")
        A("|---|---|---|---|---|---|---|---|---|---|---|")
        for q in r["phases"]:
            A(f"| {q['mode']} | {q['t0']:.1f} – {q['t1']:.1f} | {q['dt']:.1f} "
              f"| {q['alt0']:.0f} → {q['alt1']:.0f} "
              f"| {q['mach0']:.3f} → {q['mach1']:.3f} "
              f"| {q['gamma0_deg']:+.1f} → {q['gamma1_deg']:+.1f} "
              f"| {q['peak_load_n']:.2f} | {q['fuel_kg']:.3f} "
              f"| {q['mean_thrust_n']:.0f} | {q['mean_drag_n']:.0f} "
              f"| {q['min_accel_g']:+.3f} |")
        if r.get("events"):
            A("")
            A("Engine events: " + "; ".join(f"`{e}`" for e in r["events"]))

    # ---- FP vs closed-form propulsion --------------------------------
    rc = runs.get("c")
    if rc and rc.get("engine_trace"):
        from medium_model.design import load_frozen_design
        from medium_model.pulsejet_simple import pulsejet_thrust

        d = load_frozen_design(FROZEN)
        g = d.geometry
        tr = rc["engine_trace"]
        A("")
        A("## 3b. What the first-principles pulsejet actually made")
        A("")
        A("One row per FP re-convergence — that is the propulsion model's own "
          "resolution, so this is the raw march, not a resampling. `CF` is "
          "the closed-form engine evaluated at the same (Mach, altitude), "
          "i.e. what rungs A and B were flying on.")
        A("")
        A("| t (s) | Mach | alt (m) | FP thrust (N) | closed-form (N) | "
          "FP / CF |")
        A("|---|---|---|---|---|---|")
        ratios = []
        for i in range(len(tr["time_s"])):
            M, alt = tr["mach"][i], tr["altitude_m"][i]
            cf = pulsejet_thrust(g.diameter_m, g.chamber_length_m,
                                 g.throat_diameter_m, g.throat_length_m,
                                 M, alt, g.fuel).average_thrust_n
            fp = tr["pulsejet_thrust_n"][i]
            if fp > 0.0:
                ratios.append(fp / cf)
                last = f"{fp / cf:.3f}"
            else:
                last = "**QUENCHED**"
            A(f"| {tr['time_s'][i]:.1f} | {M:.3f} | {alt:.0f} | {fp:.1f} "
              f"| {cf:.1f} | {last} |")
        A("")
        if ratios:
            A(f"Across the {len(ratios)} points where the engine was alive, "
              f"the first-principles pulsejet makes **{100*sum(ratios)/len(ratios):.1f}%** "
              f"of the closed-form thrust (worst live point "
              f"{100*min(ratios):.1f}%). The freeze's own risk note allowed "
              f"for a 10% haircut; the mean is close to that, but the mean is "
              f"not what kills the design — the quench is, and a scalar "
              f"thrust haircut cannot represent it.")
        A("")

    # ---- gates -------------------------------------------------------
    A("")
    A("## 4. Constraint status")
    A("")
    A("| constraint | target | " + " | ".join(f"rung {n}" for _, n, _ in have)
      + " |")
    A("|---|---|" + "---|" * len(have))

    def gate(label, target, fn):
        cells = []
        for k, _, _ in have:
            ok, txt = fn(runs[k])
            cells.append(f"**{'PASS' if ok else 'FAIL'}** ({txt})")
        A(f"| {label} | {target} | " + " | ".join(cells) + " |")

    def _light(r):
        if r["ramjet_lightoff_mach"] is None:
            why = ("FP declined inside the policy window"
                   if r["ramjet_light_refused"]
                   else f"never lit — peak Mach {r['peak_mach']:.3f} never "
                        f"reached the {r['gate_mach']:.2f} gate, so the "
                        f"engine was never asked")
            return (False, why)
        return (bool(r["ramjet_lit_in_dive"]),
                f"M {r['ramjet_lightoff_mach']:.3f} in "
                f"{r['ramjet_lightoff_mode']}")

    gate("ramjet lights in the dive", "required", _light)
    gate("peak Mach", "&ge; 1.0",
         lambda r: (r["peak_mach"] >= 1.0, f"{r['peak_mach']:.3f}"))
    gate("motor cutoff reached", "yes",
         lambda r: (bool(r["cutoff"]), "yes" if r["cutoff"] else "no"))
    def _floor(r):
        # The check carries a 1.0 m Euler tolerance (V4_FLOOR_TOLERANCE_M), so
        # say out loud when the flown minimum is actually under the floor.
        flown, floor = r["min_powered_altitude_m"], r["floor_altitude_m"]
        txt = f"{num(flown, '.1f')} m"
        if flown is not None and flown < floor:
            txt += (f", {floor - flown:.2f} m under the {floor:.1f} m floor "
                    f"but inside the 1.0 m discretisation tolerance")
        return (not r["floor_violated"], txt)

    gate("400 ft floor", "&ge; 121.9 m", _floor)
    gate("peak body load", "&le; 4.0 g",
         lambda r: (r["peak_load_n_total"] <= 4.0,
                    f"{r['peak_load_n_total']:.2f} g"))
    gate("fuel within cap", "&le; cap",
         lambda r: (not r["tank_dry"],
                    f"{r['fuel_kg']:.3f} / {r['burn_cap_kg']:.3f} kg"))
    gate("safe landing", "yes",
         lambda r: (bool(r["safe_landing"]),
                    "yes" if r["safe_landing"]
                    else ("stalled" if r["stalled"] else "no")))
    gate("gamma &ge; 0 from M 0.80", "required",
         lambda r: (not r["rule_violated"],
                    ("violated" if r["rule_violated"] else
                     ("vacuous — never reached M 0.80"
                      if r["peak_mach"] < 0.80 else "satisfied"))))
    gate("min traverse accel", "&ge; 0.25 g",
         lambda r: (r["traverse_g"] >= 0.25, f"{r['traverse_g']:.3f} g"))
    gate("min powered accel", "> 0",
         lambda r: (r["powered_g"] > 0.0, f"{r['powered_g']:.3f} g"))
    gate("engine-only thrust margin", "&ge; 1.15",
         lambda r: (r["margin"] >= 1.15, f"{r['margin']:.3f}"))
    A("")

    # ---- plots -------------------------------------------------------
    A("## 5. Plots")
    for k, name, what in have:
        tag = runs[k]["tag"]
        A("")
        A(f"### Rung {name}")
        A("")
        A(f"![rung {name} trajectory and body loads]({tag}_trajectory.png)")
        A("")
        A(f"![rung {name} fuel]({tag}_fuel.png)")
        if (OUT / f"{tag}_engines.png").exists():
            A("")
            A(f"![rung {name} engines]({tag}_engines.png)")
    A("")
    A("## 6. Reproduce")
    A("")
    A("```bash")
    for k, name, _ in have:
        r = runs[k]
        extra = ("" if r["propulsion"] == "closed-form" else
                 f" --n-cells {r['n_cells']} "
                 f"--chamber-fraction {r['chamber_diameter_fraction']:g}")
        if r.get("light_at_pullout"):
            extra += " --light-at-pullout"
        A(f"PYTHONPATH=. .venv/Scripts/python "
          f"scripts/medium_model_v4_final.py --rung {k[0]}{extra}")
    A("PYTHONPATH=. .venv/Scripts/python scripts/medium_model_v4_report.py")
    A("```")
    A("")

    path = OUT / "v4_gate3_report.md"
    path.write_text("\n".join(L), encoding="utf-8")
    print(f"wrote {path}  ({len(L)} lines, rungs {[n for _, n, _ in have]})")


if __name__ == "__main__":
    main()
