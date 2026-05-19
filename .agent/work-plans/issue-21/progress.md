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
- [ ] (must-fix) parse_log_time rejects HH:MM:SS short form — drops today's events — `diagnostics_logic.py:437`
- [ ] (must-fix) rolling-window math + session-age use monitor's wall clock vs router log timestamps — clock skew yields wrong windows + negative ages — `diagnostics_logic.py:524-536`
- [ ] (suggestion) `_IFACE_AT_RE` regex not MAC-anchored — could match a non-MAC `@` prefix in future log formats — `diagnostics_logic.py:387`
- [ ] (suggestion) classify_event substring match is case-sensitive — silent drop on case-variant messages — `diagnostics_logic.py:409-413`
- [ ] (suggestion) double call to event_interface() in set comprehension — code smell — `mikrotik_monitor_node.py:444`
- [ ] (suggestion) /log sub-poll failure preserves prior events while poll_monotonic advances — events task renders OK with stale data — `mikrotik_monitor_node.py:327`
- [ ] (suggestion) misleading "Filter server-side" comment — actually client-side filter — `mikrotik_monitor_node.py:289`

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
- [ ] **Fix #1** (`diagnostics_logic.py:437`, must-fix): treat
  `HH:MM:SS` short-form RouterOS timestamps as "today" by composing with
  the monitor host's wall-clock date; today's events are otherwise
  invisible in the per-radio events task.
- [ ] **Fix #2 (light)** (`diagnostics_logic.py:524-536`, must-fix):
  clamp negative `Ns ago` to 0 with a once-per-skew log. Full clock-skew
  KV (`clock_skew_sec`) is nice-to-have for forensics.
- [ ] **Fix #3** (`mikrotik_monitor_node.py:327`): track
  `wireless_events_last_success_monotonic` alongside the cached value;
  surface as `events_poll_age_sec` KV. Same pattern likely applies to
  `wireless` and `system_health` — consider doing generally.
- [ ] **Fix #4** (`mikrotik_monitor_node.py:289`): correct the
  "Filter server-side" comment to reflect the actual client-side
  filtering (RouterOSClient docstring already accurate).
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
- [ ] **Fix #1 (correctness)** (`diagnostics_logic.py:670-675`): add
  `last_drop_ts.isoformat()` to the `last_drop` KV value to match the
  docstring (lines 559-578) and the `last_assoc` format. Today
  `last_drop` only carries "Ns ago: msg", missing the absolute wall
  time the docstring promises.
- [ ] **Fix #2 (correctness)** (`diagnostics_logic.py:~700`):
  `clock_skew_sec` is misleading — value is `(now_wall_dt -
  latest_event_ts)`, which is "age of latest event", not skew. Cheap
  fix: **rename to `latest_event_age_sec`**. Right fix (follow-up):
  fetch `/system/clock` from RouterOS and compute true skew.
- [ ] **Fix #3 (nit)** (`mikrotik_monitor_node.py:478-483`): replace
  the set comprehension with a `for` loop or walrus operator so
  `event_interface(e)` is computed once per event, not twice.
- [ ] **Fix #4 (nit)** (`diagnostics_logic.py:494`): change
  `classify_event` signature to `message: Optional[str]` — current
  type hint says `str` but the `if not message:` guard accepts None.
- [ ] **Fix #5 (housekeeping)** — update PR #22 description to reflect
  that short-form `HH:MM:SS` is now accepted (composed with
  `now_wall_dt`), not rejected.
- [ ] **Defer (follow-up issue)**: `synthesize_wireless_events_status`
  re-parses + filters + sorts the full /log buffer on every diagnostic
  publish. O(n log n) per Hz scales fine for now (~1000 entries) but
  warrants caching keyed on `wireless_events_last_success_monotonic`
  if buffer grows. Not blocking deployment.
