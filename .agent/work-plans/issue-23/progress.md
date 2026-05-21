---
issue: 23
---

# Issue #23 — mikrotik_monitor + teltonika_monitor crash silently on startup if initial connect fails

## External Review
**Status**: complete
**When**: 2026-05-21 14:20
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #24 — 1 review, 4 valid, 0 false positives
**CI**: all-pass (copilot-pull-request-reviewer success; no other checks configured)

### Actions
- [ ] Cap `_consecutive_failures` (or use a bounded exponent / bit-shift with clamped shift count) in both monitors so the backoff multiplication cannot raise `OverflowError` after extended outages. Verified threshold: `5.0 * (2 ** 1024)` silently produces `inf` (saved by `min()`); `5.0 * (2 ** 1025)` raises `OverflowError`. With default `poll_interval=5s` + `backoff_max_sec=60s`, that's ~17 h of continuous failure, plausible for an unattended operator station.
- [ ] Validate `backoff_max_sec >= poll_interval` at `__init__` in both monitors. Clamp (and log a warning) on misconfig so the `(retry in Xs)` operator hint stays accurate.
- [ ] (Optional, raised in the internal pre-Copilot review) extract the pure backoff math into a one-line helper in `diagnostics_logic.py` to make it unit-testable; both fixes above would then be covered by the existing pytest infrastructure.
