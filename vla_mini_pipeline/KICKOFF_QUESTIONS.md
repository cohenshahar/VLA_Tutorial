# Kickoff questions — vla_mini_pipeline

**Read by:** Claude Code, at the start of the first session on the strong GPU machine.
**Before doing anything in Phase 0, ask the questions in §1.** Block on answers.
Questions in §2 can wait until the phase that needs them.

The supervisor (Shahar) is the source of truth — do not assume defaults silently.
If an answer would change a downstream phase, restate the impact before moving on.

---

## §1 — Blocking questions (ask before any Phase 0 command)

These gate hardware setup. Wrong assumptions here can burn half a day.

### Q1. Exact GPU model and VRAM
Why it matters: chooses model (OpenVLA-7B vs Octo), batch size, and whether LoRA fine-tune is feasible at all.
- 24 GB (RTX 4090 / RTX 3090 / A5000) — OpenVLA-7B inference fine, LoRA fine-tune tight
- 40 GB+ (A6000 48 GB / A100 40 GB / A100 80 GB) — recommended, painless LoRA
- 16 GB or less — switch model to Octo, drop OpenVLA-7B from this plan
- Multi-GPU — confirm count and per-card VRAM

### Q2. OS state right now
- Fresh Ubuntu 22.04 install, nothing else done
- Ubuntu 22.04 with NVIDIA driver + CUDA already installed and verified (`nvidia-smi` works)
- Different Linux distro (Ubuntu 24.04 / Pop!_OS / other) — flag risk, OpenVLA tested on 22.04
- Cloud GPU (RunPod / Lambda / Vast.ai) — confirm image name

### Q3. HuggingFace account + access token ready?
Why it matters: OpenVLA checkpoint download. Some related models are gated.
- Yes, token already in `~/.cache/huggingface/token`
- Yes, account exists, token not yet placed — Claude Code walks Shahar through `huggingface-cli login`
- No account — pause and create one before Phase 1

### Q4. Disk space available on the partition that will hold checkpoints + datasets?
Why it matters: OpenVLA checkpoint ≈ 14 GB, LIBERO ≈ 10 GB, demo dataset 5–50 GB, intermediate fine-tune checkpoints 30–100 GB.
- ≥ 300 GB free — comfortable
- 100–300 GB free — manageable, prune old checkpoints
- < 100 GB free — stop, add storage before Phase 1

### Q5. Network: bandwidth and proxy
Why it matters: 14 GB checkpoint download + LIBERO + Python wheels.
- Open residential / lab network, no proxy, ≥ 50 Mbit/s — fine
- Behind a corporate / university proxy — confirm HTTP_PROXY / HTTPS_PROXY env vars before any `pip install` or `huggingface-cli download`
- Slow connection (< 10 Mbit/s) — checkpoint download alone is multi-hour; plan accordingly

### Q6. Path A still locked, or has anything changed?
Confirm before Phase 0: still doing Franka/LIBERO inference first, then porting to KR6?
- Yes — proceed as `PLAN.md` Phase 0 → Phase 8
- Changed — stop, return to Cowork with Shahar to revise the plan, don't improvise

---

## §2 — Phase-specific questions (ask when the phase opens)

### Phase 1 (OpenVLA on LIBERO)
- Which LIBERO suite first? (`spatial`, `object`, `goal`, `10`) — default to `spatial`, smallest scope
- Save rollout videos for every episode, or only successes?
- Where should the HuggingFace cache live? Default `~/.cache/huggingface` or a project-local dir to keep the home partition clean?

### Phase 2 (codebase reading)
- Use Jupyter notebook in `notebooks/` for trace exploration, or plain Python scripts?
- Should the codebase-trace writeup be one file or one per subsystem?

### Phase 3 (KR6 plumbing)
- Copy KR6 XML from thesis sim path or from a clean git checkout? Shahar must confirm the path.
- Single camera or a primary + wrist? Default single primary, 224×224 after resize.
- Run headless (offscreen render) or with GLFW viewer for visual confirmation? Default: viewer for first rollout, headless for batch eval.

### Phase 4 (demo collection)
- Confirm task: "pick the red cube and place it in the box"? Confirm cube/box sizes and start-pose randomization range.
- Oracle policy: borrow IK from `arm_teleop_mini/` or write fresh? Default: borrow.
- Storage format: RLDS via `tensorflow_datasets`, or HDF5? Default: RLDS, matches OpenVLA fine-tune script.
- Target demo count: 50, 100, or 200? Default 100.

### Phase 5 (LoRA fine-tune)
- Logging: W&B, TensorBoard, both, or neither? Default: both.
- LoRA rank: 16, 32, 64? Default 32.
- Checkpoint cadence: every N steps? Default every 500.
- Time budget for fine-tune wall-clock: hours allowed? Default 6 hours max for first run.

### Phase 6 (eval)
- N rollouts per condition? Default 20.
- Save all videos or only a sampled subset? Default: 3 successes + 3 failures per condition.

### Phase 7 (ablation)
- Which ablation? `language` / `data-scale` / `camera`. Default: `language` (cheapest).

### Phase 8 (wrap-up)
- Push final writeup to the `VLA_Tutorial/` GitHub repo, or keep local? Default: push.
- Anything to promote into the thesis Notes folder? (Boundary: code stays here, *learnings* can travel.)

---

## §3 — Anti-patterns Claude Code must refuse

- **Do not** `sudo` install global Python packages. Use the project conda env.
- **Do not** modify the thesis sim folder. Copy XML files, don't symlink, don't edit in place.
- **Do not** commit checkpoint files (`.safetensors`, `.pt`, `.bin`) to git. Add to `.gitignore` immediately.
- **Do not** commit HuggingFace tokens, W&B API keys, or any `.env` file.
- **Do not** silently swap models (e.g., OpenVLA → Octo) without re-confirming with Shahar.
- **Do not** skip the LIBERO win (Phase 1) and jump straight to KR6. Path A is the chosen path.

---

*VLA Tutorial / vla_mini_pipeline | 2026-05-25*
