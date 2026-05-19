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
