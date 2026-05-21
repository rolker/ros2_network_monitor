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
- [x] Cap `_consecutive_failures` (or use a bounded exponent / bit-shift with clamped shift count) in both monitors so the backoff multiplication cannot raise `OverflowError` after extended outages. Verified threshold: `5.0 * (2 ** 1024)` silently produces `inf` (saved by `min()`); `5.0 * (2 ** 1025)` raises `OverflowError`. With default `poll_interval=5s` + `backoff_max_sec=60s`, that's ~17 h of continuous failure, plausible for an unattended operator station.
- [x] Validate `backoff_max_sec >= poll_interval` at `__init__` in both monitors. Clamp (and log a warning) on misconfig so the `(retry in Xs)` operator hint stays accurate.
- [x] (Optional, raised in the internal pre-Copilot review) extract the pure backoff math into a one-line helper in `diagnostics_logic.py` to make it unit-testable; both fixes above would then be covered by the existing pytest infrastructure.

## Fix
**Status**: complete
**When**: 2026-05-21 14:35
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Commit**: `9437dcb` — added `compute_poll_backoff()` (bounded bit-shift) and `clamp_backoff_max_sec()` to each package's `diagnostics_logic.py`; wired both into the node `__init__` and `_poll_callback`; added 14 unit tests (7 per package). `colcon test`: 96/96 passing across both packages.

## External Review
**Status**: complete
**When**: 2026-05-21 14:45
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #24 — 2 reviews (1 stale, 1 fresh), 4 new valid findings, 0 false positives
**CI**: all-pass

### Actions
- [x] Reword the cached `(retry in Xs)` hint in both monitors so the displayed countdown stays accurate while the poll body is gated. The current code sets `error_message` once at failure time with `retry in {backoff}s` and then early-returns from `_poll_callback` for the next `backoff` seconds — the publish timer keeps reading that frozen value, so the operator sees a stale countdown. Minimal fix: change phrasing to non-countdown form (`backoff={backoff:.0f}s`) so the cached value is correct regardless of elapsed time. Stretch fix: refresh the cache from inside the early-return branch each timer fire with `retry in {max(0, _next_poll_monotonic - now):.0f}s`.
- [x] Capture the exception in both `main()` guards and include `{type(exc).__name__}: {exc}` in the fatal log so `/rosout` shows *why* the node failed to construct, not just *that* it did. Currently `except Exception:` discards the value; the fatal message is the only operator-visible artifact pre-Node-startup.

## Fix
**Status**: complete
**When**: 2026-05-21 14:55
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**Commit**: `8842b44` — reworded the cached error_message in both nodes from `retry in {backoff}s` to `backoff {backoff}s` (window, not countdown) so the operator's panel reading stays accurate during the gated early-return; captured `as exc` in both `main()` guards and interpolated `{type(exc).__name__}: {exc}` so `/rosout` shows the cause. `colcon test`: 96/96 still passing.
