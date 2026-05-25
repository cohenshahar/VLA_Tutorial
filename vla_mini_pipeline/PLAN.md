# vla_mini_pipeline — Project Plan

**Owner**: Shahar Cohen
**Created**: 2026-05-25
**Status**: PLAN (pre-hardware)
**Repo location**: `VLA_Tutorial/vla_mini_pipeline/` (when created)
**Per CLAUDE.md §7 rule #4**: Practice infrastructure only — **not thesis content**.
**Path chosen**: Path A — Franka/LIBERO quick win first, then KR6 mini-pipeline.

---

## 1. Mission

Learn the OpenVLA stack end-to-end by going through a complete mini research loop on a strong GPU machine:

> **pretrained inference → understand codebase → port to KR6 → collect demos → LoRA fine-tune → evaluate → one ablation → write up**

Goal is **understanding**, not a result for the thesis. Every step must leave Shahar able to explain what OpenVLA reads, what it outputs, and where each component lives in the code.

## 2. Why this project (and what it's NOT)

**What it teaches**:

- The OpenVLA architecture in working code (VLM backbone, action de-tokenizer, observation pipeline)
- The LIBERO eval loop and what a "VLA rollout" actually is
- The RLDS / open-x-embodiment data format
- Realistic image-observation plumbing (resolution, normalization, camera pose)
- LoRA fine-tuning on a robotics foundation model
- A clean eval (pretrained vs fine-tuned, success rate over N rollouts)
- One ablation (probably language conditioning or demo-count)

**What it is NOT**:

- Not thesis content (lives in `VLA_Tutorial/vla_mini_pipeline/`, not the thesis repo)
- Not the Continuous Verifier (the thesis direction is untouched by this work)
- Not a SOTA chase — fine-tune quality only needs to be "obviously better than pretrained baseline"
- Not a replacement for `arm_teleop_mini` or for `demo_reach.py` (Phase 10.1)

**Hard ceiling**: **4 weeks from hardware arrival.** If we're not at Phase 6 (eval) by Week 3, we ship Phase 1–3 as the learning win and return to thesis main track. This protects against §8 trigger #4 (three weeks no thesis movement).

## 3. Today's deliverable (2026-05-25)

There is no GPU in hand, so today's deliverable is **this plan + a hardware spec**, not a running arm. Once the box lands, Phase 0 starts.

## 4. Hardware spec (Phase 0 input)

Minimum acceptable build to execute this plan as written:

| Component | Recommended | Why |
|---|---|---|
| **GPU** | RTX 4090 24GB **or** RTX A6000 48GB | OpenVLA-7B inference needs ~16 GB in bf16; LoRA fine-tune needs 24–40 GB. A6000 makes Phase 5 painless. |
| **CPU** | Any modern 8+ core (Ryzen 7 / i7) | Sim physics is single-threaded-ish; CPU isn't the bottleneck. |
| **RAM** | 64 GB | RLDS dataset shards eat RAM; LIBERO loads ~10–20 GB at peak. |
| **Storage** | 1 TB NVMe SSD | OpenVLA checkpoint ≈ 14 GB, LIBERO ≈ 10 GB, dataset shards 50–200 GB. |
| **OS** | Ubuntu 22.04 LTS | OpenVLA is tested here; Ubuntu 24.04 sometimes breaks NVIDIA driver chain. |

**Cloud fallback** (if hardware is delayed): RunPod or Lambda with an A6000 / A100 80GB instance. ~$0.50–1.50/hr. Same plan, skip Phase 0.1–0.3. Useful for time-boxing the learning project to a focused weekend if hardware drifts.

## 5. Phase plan

Each phase has: **(a)** scope, **(b)** deliverable, **(c)** "Done = visible thing", **(d)** estimated wall-clock time.

### Phase 0 — Hardware & environment (Day 1 = arrival day, 4–8h)

**Scope**: get from boxed PC to `torch.cuda.is_available() == True`.

0.1 Install Ubuntu 22.04 LTS, full updates, openssh-server.
0.2 Install NVIDIA driver (latest stable from `ubuntu-drivers autoinstall` or `apt install nvidia-driver-550`). Reboot. `nvidia-smi` shows the GPU.
0.3 Install CUDA toolkit 12.1 (matches PyTorch 2.4 wheels).
0.4 Install miniforge / mambaforge.
0.5 Create env: `mamba create -n vla python=3.10`.
0.6 Install PyTorch with CUDA: `pip install torch==2.4.0 torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121`.
0.7 Verify: `python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"`.
0.8 Install dev basics: `git`, `tmux`, `htop`, `nvtop`, `code` (VS Code), `ffmpeg`.

**Done = visible**: `nvidia-smi` + PyTorch print both show your GPU.

### Phase 1 — OpenVLA pretrained inference on LIBERO (Day 2, 4–8h)  ← "see the arm"

**Scope**: a Franka in LIBERO executes a natural-language pick-and-place under OpenVLA-7B with no training of our own.

1.1 Clone OpenVLA: `git clone https://github.com/openvla/openvla.git`.
1.2 Install per their README (transformers, flash-attn, accelerate, peft, draccus, etc.).
1.3 Install LIBERO: follow `openvla/experiments/robot/libero/README.md`.
1.4 Download checkpoint: `openvla/openvla-7b` from HuggingFace. Cache to a known dir.
1.5 Run their LIBERO eval on one task (e.g., `libero_spatial`).
1.6 Watch the rendered video / save a few rollout MP4s.

**Done = visible**: an .mp4 showing a Franka picking a block under language command, with the success/failure printed.
**This is the moment you see "arm in VLA."**

### Phase 2 — Understand the codebase (Day 3–4, ~6h)

**Scope**: read end-to-end the path from `obs` → `action`. No code changes.

2.1 Trace the inference loop from `experiments/robot/libero/run_libero_eval.py` into the model.
2.2 Map the observation pipeline: image source → resize 224×224 → normalize → tokenize.
2.3 Map the action pipeline: VLM logits → discrete action tokens → 7-DoF continuous action chunk.
2.4 Identify where language is injected (the prompt template) and which tokenizer is in play.
2.5 Run two more LIBERO suites (`object`, `goal`) to see how task variety changes success rate.

**Deliverable**: a 1–2 page `Research_Note_<date>.md` in the project repo explaining the OpenVLA inference loop in your own words. **This is the actual learning artifact.** If you can't write this, you don't yet understand it.

### Phase 3 — KR6 plumbing (Week 2, ~3–5 days)

**Scope**: get the KR6 MuJoCo scene from your Phase 9 thesis work running as an OpenVLA-compatible env.

3.1 Copy (don't move — thesis sim stays in place) the KR6 MuJoCo XML into `vla_mini_pipeline/sim/`.
3.2 Add 1 RGB camera at the right resolution. Match OpenVLA's expected input (224×224 after resize).
3.3 Write a thin `KR6Env` wrapper exposing `.reset()` and `.step(action)`, with `obs = {"image": HxWxC uint8, "language_instruction": str}`.
3.4 Run pretrained OpenVLA-7B on `KR6Env` for one episode. **Expect garbage** — the arm will twitch. This is the correct, expected baseline result.
3.5 Save a rollout video as the "pretrained baseline on KR6" reference clip.

**Done = visible**: an .mp4 of KR6 twitching under OpenVLA. The point is to see the baseline before fine-tuning.

### Phase 4 — Demo collection (Week 2–3, ~3–5 days)

**Scope**: build a small dataset of (image, language, action) tuples for one task.

4.1 Define ONE task. Suggested: "pick the red cube and place it in the box." Single object, single goal, fixed start pose distribution with light randomization.
4.2 Write a scripted oracle policy in MuJoCo (use IK from `arm_teleop_mini` if useful). 50–200 successful demos.
4.3 Save each demo as a sequence of `(image_t, action_t, language)` records.
4.4 Store in RLDS format (use `tensorflow_datasets` per OpenVLA's data docs). One TFDS dataset = one shard for the start.
4.5 Sanity-check the dataset by reloading and visualizing one demo as a video.

**Done = visible**: a folder of RLDS shards on disk, and one playback video that proves the data round-trips correctly.

### Phase 5 — LoRA fine-tune (Week 3, ~2–4 days)

**Scope**: fine-tune OpenVLA-7B on the KR6 dataset with LoRA. Don't full-fine-tune.

5.1 Use OpenVLA's fine-tuning script (`vla-scripts/finetune.py`). Configure: dataset path, LoRA rank=32, target Q/K/V/O projections.
5.2 Start with a tiny run: 500 steps, batch size 4–8, learning rate 5e-4. Time it.
5.3 If loss curve looks sane, scale to ~5–20k steps depending on GPU and dataset size.
5.4 Monitor: loss curve, gradient norms, learning rate schedule. Save a checkpoint every N steps.

**Done = visible**: a TensorBoard / wandb loss curve trending down, and a saved LoRA checkpoint file.

### Phase 6 — Evaluation (Week 3–4, ~2 days)

**Scope**: compare pretrained vs fine-tuned on `KR6Env`.

6.1 Run pretrained OpenVLA on the KR6 task for N=20 rollouts. Record success rate.
6.2 Run fine-tuned OpenVLA (LoRA merged or loaded) for N=20 rollouts. Record success rate.
6.3 Compute success rate, mean episode length, basic failure mode breakdown (e.g., grasp miss / drop / overshoot).
6.4 Save 3 success videos + 3 failure videos for the writeup.

**Done = visible**: a 2-column table — `pretrained: X% / fine-tuned: Y%` — with Y >> X. If they're equal, something is broken; treat as an `engineering:incident-response` situation.

### Phase 7 — One ablation (Week 4, ~1–2 days)

**Scope**: one small ablation to learn how the system reacts to a change.

Pick ONE:
- **Language ablation**: re-eval the fine-tuned model with shuffled / empty / wrong language. Does success rate drop?
- **Data-scale ablation**: fine-tune separately on 50 / 100 / 200 demos. Plot success rate vs demo count.
- **Camera ablation**: re-eval with a slightly perturbed camera pose. How brittle is the policy?

**Done = visible**: a single plot or 2-row table answering one question.

### Phase 8 — Wrap-up (Week 4, ~half a day)

8.1 Write a `Research_Note_<date>.md` documenting: what you built, what worked, what didn't, what you'd do differently.
8.2 List which pieces of code are worth keeping in `VLA_Tutorial/` vs deleting.
8.3 Flag any **incidental thesis learnings** (e.g., "fine-tuning a VLA is harder than I expected because X" — that goes in the thesis Notes folder as a separate file, with the boundary clearly marked).
8.4 Delete or archive intermediate checkpoints and dataset copies. Storage hygiene.

**Done = visible**: the writeup file exists in `VLA_Tutorial/vla_mini_pipeline/` and is readable by future-you in 3 months.

## 6. Timeline summary

| Week | Phases | Visible milestone |
|---|---|---|
| Hardware arrival day | Phase 0 | `torch.cuda.is_available()` ✓ |
| Week 1 (Days 2–5) | Phases 1–2 | Franka in LIBERO under OpenVLA, codebase notes |
| Week 2 | Phases 3–4 (start) | KR6 baseline video (twitchy), demo collection underway |
| Week 3 | Phases 4 (finish) – 5 – 6 (start) | Dataset shipped, fine-tune running, eval scaffolding ready |
| Week 4 | Phases 6 (finish) – 7 – 8 | Success-rate table, one ablation plot, writeup |

**Total ≈ 4 weeks from hardware arrival.** Hard ceiling per §2.

## 7. Risks and decision points

| Risk | Trigger | Response |
|---|---|---|
| OpenVLA install hell on Phase 1 | >1 day stuck on dependencies | Switch to RunPod with a pre-built OpenVLA container. Don't burn 3 days on flash-attn. |
| KR6 + OpenVLA observation mismatch | Phase 3 rollout returns NaN actions | `engineering:debug` skill, check image dtype/range/shape before model. |
| Fine-tune doesn't improve baseline | Phase 6 shows Y ≈ X | Three likely causes: dataset too small (<50 demos), task too hard for the architecture, wrong action normalization. Open an `engineering:incident-response`. |
| Time drift | End of Week 3 without Phase 6 done | Stop. Ship Phases 1–3 as the learning win. Move back to thesis main track. |
| Scope creep into thesis | Tempted to "use the verifier idea here" | **No.** This project's contract is learning, not contribution. Park ideas in `Thesis/direction_candidates/` if useful. |

## 8. Cross-links

- `arm_teleop_mini_plan.md` — sibling learning project, may donate IK code to Phase 4 oracle policy
- `Simulation/sim_work_plan.md` — thesis sim work; **do not modify** from this project
- `Papers/status_log.md` — OpenVLA paper is already read (CLAUDE.md §5)
- Main thesis direction (Continuous Verifier) — untouched by this project

## 9. Approval

This plan needs Shahar's explicit sign-off before any work starts on Day 1 (per CLAUDE.md §7 rule #6).

---

*VLA Tutorial / Learning sub-project | Shahar Cohen | BGU Mechatronics | 2026-05-25*
