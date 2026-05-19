---
issue: 21
---

# Issue #21 — mikrotik_monitor: surface wireless events (associations, drops, signal) as observable diagnostics

## Local Review
**Status**: complete
**When**: 2026-05-18 20:52
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))
**Verdict**: changes-requested

**PR**: #22 at `58e9fab`
**Mode**: post-PR
**Depth**: Standard (reason: medium-size feature, ROS package code)
**Must-fix**: 2 | **Suggestions**: 5

### Findings
- [x] (must-fix) parse_log_time rejects HH:MM:SS short form — drops today's events — `diagnostics_logic.py:437` — **fixed in `91303f7`** (composes short form with `now_wall_dt`'s date, rolls back across midnight)
- [x] (must-fix) rolling-window math + session-age use monitor's wall clock vs router log timestamps — clock skew yields wrong windows + negative ages — `diagnostics_logic.py:524-536` — **fixed in `91303f7`** (clamp Ns ago to ≥0; expose `latest_event_age_sec` KV, renamed from `clock_skew_sec` in `5168ca5`)
- [ ] (suggestion) `_IFACE_AT_RE` regex not MAC-anchored — could match a non-MAC `@` prefix in future log formats — `diagnostics_logic.py:387` — **deferred** (no field observation yet)
- [ ] (suggestion) classify_event substring match is case-sensitive — silent drop on case-variant messages — `diagnostics_logic.py:409-413` — **deferred** (no field observation yet)
- [x] (suggestion) double call to event_interface() in set comprehension — code smell — `mikrotik_monitor_node.py:444` — **fixed in `5168ca5`** (walrus operator)
- [x] (suggestion) /log sub-poll failure preserves prior events while poll_monotonic advances — events task renders OK with stale data — `mikrotik_monitor_node.py:327` — **fixed in `91303f7`** + generalized to all sub-queries in `80bc3e2`
- [x] (suggestion) misleading "Filter server-side" comment — actually client-side filter — `mikrotik_monitor_node.py:289` — **fixed in `91303f7`**

## External Review
**Status**: complete
**When**: 2026-05-19 10:10
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #22 — 1 Copilot review, 4 inline comments, **4 valid**, 0 false positives.
Reviewed against `58e9fab`; only commit since is a progress.md update so all
4 still apply to current head `165fb68`.
**CI**: no checks configured on this repo.

### Corroboration with local review
All 4 Copilot findings overlap with the prior local-review findings above —
both must-fix items (#1 short-form timestamp, #2 clock skew) and 2 of the 5
suggestions (#3 stale events on /log failure, #4 misleading server-side
comment). Independent confirmation that the gaps are real.

### Actions
- [x] **Fix #1** (`diagnostics_logic.py:437`, must-fix): treat
  `HH:MM:SS` short-form RouterOS timestamps as "today" by composing with
  the monitor host's wall-clock date; today's events are otherwise
  invisible in the per-radio events task. — **landed in `91303f7`**
- [x] **Fix #2 (light)** (`diagnostics_logic.py:524-536`, must-fix):
  clamp negative `Ns ago` to 0 with a once-per-skew log. Full clock-skew
  KV (`clock_skew_sec`) is nice-to-have for forensics. — **landed in
  `91303f7`** (KV later renamed to `latest_event_age_sec` in `5168ca5`)
- [x] **Fix #3** (`mikrotik_monitor_node.py:327`): track
  `wireless_events_last_success_monotonic` alongside the cached value;
  surface as `events_poll_age_sec` KV. Same pattern likely applies to
  `wireless` and `system_health` — consider doing generally. — **landed
  in `91303f7`** + generalized to all sub-queries in `80bc3e2`
- [x] **Fix #4** (`mikrotik_monitor_node.py:289`): correct the
  "Filter server-side" comment to reflect the actual client-side
  filtering (RouterOSClient docstring already accurate). — **landed in
  `91303f7`**
- [ ] (Optional) Dismiss the Copilot review on #22 once fixes land.

## External Review (round 2 — re-review after fixes pushed)
**Status**: complete
**When**: 2026-05-19 15:05
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #22 at `80bc3e21` — 4 additional Copilot review rounds requested,
21 total inline comments across 5 rounds. Deduplicated against the
addressed-by-prior-commits set: **6 net-new findings, 0 false positives,
2 real correctness concerns, 4 nits**.
**CI**: copilot-pull-request-reviewer success (no other CI configured).

### Actions
- [x] **Fix #1 (correctness)** (`diagnostics_logic.py:670-675`): add
  `last_drop_ts.isoformat()` to the `last_drop` KV value. — **landed in
  `5168ca5`**
- [x] **Fix #2 (correctness)** (`diagnostics_logic.py:~700`):
  rename `clock_skew_sec` to `latest_event_age_sec`. — **landed in
  `5168ca5`**. Right fix (fetch `/system/clock`) tracked as deferred
  below.
- [x] **Fix #3 (nit)** (`mikrotik_monitor_node.py:478-483`): walrus
  operator. — **landed in `5168ca5`**
- [x] **Fix #4 (nit)** (`diagnostics_logic.py:494`):
  `classify_event(message: Optional[str])`. — **landed in `5168ca5`**
- [x] **Fix #5 (housekeeping)** — PR #22 description updated via
  `gh pr edit`.
- [ ] **Defer (follow-up issue)**: `synthesize_wireless_events_status`
  re-parses + filters + sorts the full /log buffer on every diagnostic
  publish. O(n log n) per Hz scales fine for now (~1000 entries) but
  warrants caching keyed on `wireless_events_last_success_monotonic`
  if buffer grows. **Not blocking deployment.** Track as follow-up issue
  if buffer growth becomes operational. Includes the "true clock skew
  via `/system/clock`" follow-up too.

## External Review (round 3 — convergence)
**Status**: complete
**When**: 2026-05-19 11:50
**By**: Claude Code Agent (Claude Opus 4.7 (1M context))

**PR**: #22 at `5168ca5a` — 6 Copilot review rounds total, 22 inline
comments across the full series. **Round 6 ran against current HEAD and
returned just 1 comment**, which targets this very `progress.md` file
flagging that the unchecked boxes were out of sync with the merged
fixes. **No remaining code-side findings.** Convergence reached.
**CI**: copilot-pull-request-reviewer success.

### Actions
- [x] Update `progress.md` to check off addressed items (closes
  the round-6 finding).
- [ ] (Optional) Dismiss the 5 prior Copilot reviews on the PR — all
  superseded by the current code state.
