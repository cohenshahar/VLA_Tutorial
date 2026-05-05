---
name: agent-dumber
description: "Slow down and solve problems step by step, as simple as possible. Use when: stuck in a loop, fix not working, keep trying the same thing, something is broken and unclear why, camera is wrong, orientation is off, physics is weird, need to debug from scratch, too many assumptions, let's go dumb, simplify, break it down, one step at a time, minimal test."
argument-hint: "Describe the problem you are stuck on"
---

# Agent Dumber

Stop. Slow down. Solve it like a scientist — one tiny experiment at a time, no assumptions, human confirms every step.

This is how we fixed the cameras: instead of patching all 3 at once, we built a scene with two colored boxes and one question: *which one does the camera see?* One test. One answer. Then fix.

---

## When to Use
- You've tried to fix something 2+ times and it's still wrong
- The fix feels like guessing
- There are multiple possible causes and you don't know which one is the real problem
- The human has a suggestion — follow it exactly before doing anything else

---

## Protocol

### Step 1 — Stop and define the ONE thing that is wrong
Ask the human (or state clearly):
> "The exact thing that is broken is: ___"

Do not list 5 things. Pick one. The most visible, most testable one.

Example:
- ❌ "The cameras are not working correctly"
- ✅ "The overhead camera is showing the ceiling instead of the floor"

---

### Step 2 — Strip everything away
Build the smallest possible test that isolates the problem:
- No arm, no full scene — unless the problem requires it
- No gravity unless needed
- Replace unknowns with colored boxes, print statements, or hardcoded values
- One variable changes per test

Example (camera direction):
```python
# No arm. No table. Just camera + 2 boxes.
# GREEN above camera. BLUE below camera.
# Question: which one appears in the render?
```

---

### Step 3 — Ask the human one question
After the test runs, show the result and ask ONE question:

> "The render shows [X]. Does that match what you expected?"

Or present two options:
> "Option A shows [X], Option B shows [Y]. Which is correct?"

Do NOT proceed to fix anything yet. Wait for the human's answer.

---

### Step 4 — Apply the minimum change
Once the human confirms the diagnosis, apply the **smallest possible fix**:
- Change one value
- Flip one sign
- Move one parameter
- Do not refactor, do not touch unrelated code

Then re-run the same test (not the full system).

---

### Step 5 — Verify before moving on
Run the isolated test again. Confirm the fix works in the small case.

Only after it passes: apply the same fix to the real scene and test again.

---

### Step 6 — Repeat for the next problem
Go back to Step 1 with the next broken thing.
One problem at a time. Never fix two things simultaneously.

---

## Rules
- **Never guess**. If you don't know which of two things is causing the problem, test both separately.
- **Human suggestions go first**. If the human says "try X", do X exactly — before any other approach.
- **Show before fixing**. Always show the test result to the human before applying the real fix.
- **One change per iteration**. If you change two things at once and it breaks, you don't know which one caused it.
- **Print/render everything**. Make the intermediate state visible — a picture, a number, a printed value.
- **No cleanup until it works**. Don't refactor, rename, or tidy code while debugging. Fix first, clean later.

---

## Example — Camera Orientation Debug (this session)

| Step | What we did |
|------|------------|
| Defined problem | "Overhead camera shows ceiling, not floor" |
| Stripped scene | No arm, no table, just camera + green box above + blue box below |
| Asked one question | "Which box appears in the render?" → Green (above) appeared |
| Diagnosed | Camera is pointing UP, not DOWN |
| Minimum test | Tried `xyaxes="1 0 0  0 1 0"` only → Blue appeared ✓ |
| Applied fix | Changed only that one line in cage.xml |
| Verified | Re-rendered full scene → overhead now looks down ✓ |
| Next problem | Side camera upside down → repeat from Step 1 |

---

## The Mindset

> "If you can't explain why the fix works, it's not a fix — it's a guess."

The goal is not speed. The goal is understanding.
A dumb test that teaches you something beats a clever patch that might break tomorrow.
