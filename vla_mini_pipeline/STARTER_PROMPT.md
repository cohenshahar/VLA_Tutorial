# Starter prompt — vla_mini_pipeline

**For Shahar.** Paste the block below into Claude Code on the strong GPU machine
at the very first session. Run it from the cloned `VLA_Tutorial/` repo root.

---

## Paste this into Claude Code (Day 1, first message)

```
You are picking up a new learning sub-project called vla_mini_pipeline.

Before doing ANYTHING:

1. Read VLA_Tutorial/vla_mini_pipeline/README.md
2. Read VLA_Tutorial/vla_mini_pipeline/PLAN.md in full
3. Read VLA_Tutorial/vla_mini_pipeline/KICKOFF_QUESTIONS.md in full
4. Read the parent thesis CLAUDE.md (the one in the VLA Thesis workspace)
   — pay attention to §4 (Cowork ≠ Ubuntu — you are the Ubuntu/strong-PC side),
   §7 rule #4 (this is learning infra, not thesis content),
   and §7 rule #1 (no invention without context).

Then:

5. Ask me every blocking question in §1 of KICKOFF_QUESTIONS.md, one by one.
   Do not assume defaults. Do not start Phase 0 commands until I answer all of §1.
6. Once §1 is answered, summarize back to me: "Plan locked, GPU is X,
   OS is Y, HF token is Z, disk is W, network is V, Path A is confirmed."
7. Only then begin Phase 0 from PLAN.md.

Rules you must follow:
- This is a learning project. Goal is understanding, not SOTA results.
- Hard ceiling: 4 weeks of wall-clock from today. If we're not at Phase 6 by
  the end of Week 3, stop and ship what we have.
- After each phase, update VLA_Tutorial/vla_mini_pipeline/HANDOFF.md
  (create it if missing) — same pattern as arm_teleop_mini/arm_teleop_handoff.md.
- Never commit checkpoints, datasets, tokens, or .env files to git.
- Never modify the thesis sim folder. Copy KR6 XML, don't edit in place.
- If a phase deliverable is unclear, ask. Don't invent.

Start with step 1.
```

---

## What to expect from Claude Code after you paste this

1. Claude Code reads the four files (a few minutes of tool calls).
2. Claude Code asks you Q1 through Q6 from `KICKOFF_QUESTIONS.md` §1 — one at a time.
3. Claude Code recaps the answers.
4. Claude Code proposes the exact Phase 0 commands (driver install, CUDA, conda env, PyTorch verify) and asks for approval before running any `sudo` or `apt` command.
5. By end of Day 1: `nvidia-smi` works, `torch.cuda.is_available() == True`. That's Phase 0 done.
6. Day 2 begins Phase 1 (OpenVLA inference on LIBERO). You should *see the arm* by end of Day 2.

## If something goes wrong on Day 1

Two failure modes are common at this stage:

- **NVIDIA driver vs kernel mismatch** — `nvidia-smi` returns an error. Reboot first. If still broken, paste the exact error back into Claude Code and let it walk the fix.
- **CUDA / PyTorch version mismatch** — `torch.cuda.is_available()` returns False even though `nvidia-smi` works. Means PyTorch wheel doesn't match CUDA. Reinstall PyTorch with the right `--index-url`.

If Claude Code can't fix in one round, come back to Cowork (this machine) with the error message and we'll triage.

---

## Where this fits

| Machine | Role in this project |
|---------|----------------------|
| Cowork (this one) | Planning, approval, post-mortem of phase results. **No commands run here.** |
| Strong GPU PC | All actual work — Phase 0 through Phase 8. Claude Code runs here. |
| Ubuntu (KR6 thesis sim) | Untouched. KR6 XML gets copied to the strong PC; thesis sim stays as-is. |

---

*VLA Tutorial / vla_mini_pipeline | 2026-05-25*
