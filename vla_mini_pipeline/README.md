# vla_mini_pipeline

**Status:** PLAN (pre-hardware) | **Created:** 2026-05-25
**Per CLAUDE.md §7 rule #4:** Practice infrastructure only — *not thesis content.*

A learning sub-project: go end-to-end through a small OpenVLA loop
(pretrained inference → understand → port to KR6 → demos → LoRA fine-tune → eval → ablation → writeup).
The goal is **understanding**, not a thesis result. Hard ceiling: **4 weeks from hardware arrival.**

## Files in this folder

| File | Purpose |
|------|---------|
| `PLAN.md` | Full phase-by-phase plan (Phase 0 → Phase 8). Read this first. |
| `KICKOFF_QUESTIONS.md` | Questions Claude Code (on the strong GPU machine) must ask Shahar at session start before any Phase 0 work begins. Blocks unsafe defaults. |
| `STARTER_PROMPT.md` | The exact text Shahar pastes into Claude Code on Day 1 to kick the project off. |

## Workflow

1. **Now (Cowork machine):** plan reviewed, approved, committed to GitHub.
2. **Day 1 (strong GPU machine, Claude Code session):** Shahar pastes `STARTER_PROMPT.md` into Claude Code. Claude Code reads `PLAN.md` + `KICKOFF_QUESTIONS.md`, asks the blocking questions, then begins Phase 0.
3. **Each session end:** Claude Code updates a `HANDOFF.md` in this folder (same pattern as `arm_teleop_mini/arm_teleop_handoff.md`).
4. **Each session start:** Claude Code reads `HANDOFF.md` first.

## What this project is NOT

- Not the thesis. The Continuous Verifier direction (see top-level `CLAUDE.md` §3) is untouched.
- Not the KR6 thesis sim. We *copy* the KR6 MuJoCo XML in; we do not modify the thesis sim.
- Not a SOTA chase. "Obviously better than pretrained baseline on KR6" is enough.

## Boundary with thesis

Anything learned here that's useful for the thesis is logged as a one-line note in
the thesis `Notes/` folder, marked `[from vla_mini_pipeline]`. Code stays here.

---

*VLA Tutorial / Learning sub-project | Shahar Cohen | BGU Mechatronics | 2026-05-25*
