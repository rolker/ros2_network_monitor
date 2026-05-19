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
