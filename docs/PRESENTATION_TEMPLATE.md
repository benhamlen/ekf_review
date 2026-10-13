# End-of-day presentation template

Brian's ask: **"what you did, why you did it, and what worked and didn't work."** The audience is the whole team. Fill this in from the day's decision log (`ONSITE_LOG.md`). Target about 12–15 minutes of talk plus Q&A. Adjust if they give you a time limit.

**Prep rules:**
- Code freeze **60–90 minutes before** the talk. Then collect plots and numbers, fill this in, and rehearse once out loud.
- Each slide gets one message, and one number or plot that backs it up.
- Mark every statement as **confirmed** or **assumed**.
- Showing the work running live is great, but have **screenshots as backup**.

---

## 1. Problem (1 min)
- The task as given: ___
- What success meant: ___ (accuracy? robustness? a working node?)
- Constraints and assumptions: ___ (sensors, rates, frames; **confirmed vs assumed**)

## 2. Approach and why (3 min)
- Architecture diagram (nodes, topics, frames, data flow): ___
- Key decision 1: ___ **because** ___ (alternatives considered: ___)
- Key decision 2: ___ **because** ___
- What I deliberately left out of scope, and why: ___

## 3. What I built (2–3 min, demo or screenshots)
- Components: ___
- How to run it: ___
- The one thing worth showing live: ___

## 4. Results (3 min)
- Baseline vs mine (e.g. dead reckoning vs EKF): ___ m / ___ rad
- Error vs ±3σ plot: ___
- Consistency (NEES/NIS): mean ___, expected ___, meaning ___
- Any surprise: ___

## 5. What worked / what didn't (2–3 min). Be candid; this is what they asked for.
| Worked | Evidence |
|---|---|
| ___ | ___ |

| Didn't work / limitation | Root cause | What I'd do |
|---|---|---|
| ___ | ___ | ___ |

## 6. Next steps (1–2 min)
- Next day: ___
- Next week: ___ (e.g. 3D error-state EKF, delayed measurements, tuned Q from specs)
- Toward flight: ___ (fault detection, degeneracy handling, validation, compute budget)

## 7. Backup slides (only if asked)
- Math: state, F, H, Q/R choices
- Frames / TF tree
- Degeneracy of terrain registration
- How I'd add sensor X (see `ADDING_SENSORS.md`)

---

## Likely questions (rehearse a 30-second answer for each)
1. Why an EKF and not a UKF, particle filter, or factor graph?
2. How did you choose Q and R? How would you validate them on real hardware without ground truth? *(NIS)*
3. What happens when terrain is flat or repetitive? How would you detect it? *(information-matrix eigenvalues)*
4. How does this change in 3D? *(error-state EKF on SO(3), ⊞/⊟)*
5. What fails first on the real rover? *(slip, timing/sync, extrinsics, lighting, dust)*
6. How do you handle delayed or out-of-order measurements?
7. How does the star tracker give you heading in the local frame? *(inertial attitude + Moon ephemeris + position)*
8. What would you test before trusting this on the Moon?
9. If you had one more day, what would you do first, and why that?
