# Plan: Retrofit ping/mikrotik/teltonika monitors to `diagnostic_updater.Updater`

## Issue

https://github.com/rolker/ros2_network_monitor/issues/15

## Context

The three monitor nodes (`ping_monitor_node`, `mikrotik_monitor_node`,
`teltonika_monitor_node`) use a hand-rolled cache-and-republish pattern
(PR #14): a poll timer caches a freshly-built `DiagnosticArray`; a publish
timer re-emits the cache and downgrades level to `STALE` when the cache
ages past `max_data_age`.

**Bugs this pattern has, verified in the code this session:**

1. **Schema drift across connection state.** In `mikrotik_monitor_node.poll_callback`
   and `teltonika_monitor_node.poll_callback`, the `except ... ClientError`
   branch appends a single summary status named `MikroTik: <hwid>` or
   `Teltonika: <hwid>` (no `: <task>` suffix), while the happy path emits
   per-task names (`: system`, `: interface/<name>`, `: cellular`,
   `: mwan3/<name>`, etc.). Downstream aggregators register whichever set
   arrives first and hold the other as a permanent orphan when state flips.
   `ping_monitor_node` does not have this bug (fixed target list, stable names).
2. **Two patterns in one workspace.** `starlink_stats_ros` PR #5 moved to
   `diagnostic_updater.Updater` for the same timing problem. Keeping the
   hand-rolled pattern here costs consistency.

The Starlink Updater implementation
(`starlink_stats_ros/starlink_stats/starlink_diagnostics_node.py`) is the
proven template: atomic `CachedStatus` swap, tasks snapshot the cache at
entry, `stale_timeout_sec` controls STALE short-circuit, `last_query_time`
KeyValue added via shared `_apply_common_kv`.

## Approach

### 1. Shared pattern (mirror Starlink)

Each monitor node gets the same structural rewrite:

- Introduce a `CachedStatus` (or similar) dataclass holding the last
  parsed response, `poll_monotonic`, `poll_wall_iso`, and
  `error_message: Optional[str]`. The cache is replaced by whole-object
  assignment under a lock so reads never see torn values.
- `poll_callback` continues to run on its own timer, performs the remote
  call, builds a new `CachedStatus`, and swaps it in atomically. **No
  `DiagnosticStatus` is built in the poll path.**
- `diagnostic_updater.Updater` at 1 Hz (config param) drives per-task
  callbacks. Each task callback snapshots `self._cache` at entry and
  renders OK/WARN/ERROR/STALE from the snapshot.
- Shared helpers: `_apply_common_kv(stat, cache, task_name)` adds
  `last_query_time` and any other shared fields; `_short_circuit_stale`
  emits `DiagnosticStatus.STALE` when `cache.status is None` or data is
  older than `stale_timeout_sec`.
- The `error_message` path that used to rename the status is gone. On
  connection failure the error is written into `cache.error_message` and
  surfaced via the connection-health task; other tasks keep reporting
  from the last-known cache until they age into STALE.

### 2. Task layout per monitor — preserve existing name schema

Current downstream annunciator configs match diagnostic names by
substring/prefix. Keep the `<Prefix>: <hwid>: <task>` convention so
configs don't break.

**ping_monitor** (one task per target; no schema change from current output):

| Task name | Level source |
|---|---|
| `Ping: <hwid>: <target_name>` (one per target) | OK if reachable and loss ≤ warn_threshold; WARN if loss > threshold; ERROR if unreachable; STALE if cache stale |

No dynamic membership — targets are a static parameter at launch.

**mikrotik_monitor**:

| Task name | Level source |
|---|---|
| `MikroTik: <hwid>: connection` | OK if last poll succeeded; ERROR with `cache.error_message` if last poll failed; STALE if both tasks unset (node just started) |
| `MikroTik: <hwid>: system` | OK from cached `system/resource` dict; STALE if cache old |
| `MikroTik: <hwid>: interface/<name>` | Per-interface dynamic set — see §3 |
| `MikroTik: <hwid>: wireless/<name>` | Per-wireless-registration dynamic set — see §3 |

Note the **new** `connection` task. It replaces the bug-creating
"rename-on-error" pattern. A workspace grep (2026-04-23) across
`layers/main/**/*.{yaml,yml}` confirmed **no downstream config exact-matches
the bare `MikroTik: <hwid>` / `Teltonika: <hwid>` summary name** —
consumers use either `startswith` prefix matching (the aggregator at
`bizzyboat_project11/config/diagnostics.yaml`) or task-suffixed names
like `: cellular` / `: wireless/` (the panel at
`bizzyboat_project11/config/bizzyboat_annunciator.yaml`). Introducing
`: connection` and dropping the bare-summary error path is strictly
additive from the consumer's perspective.

**teltonika_monitor**:

| Task name | Level source |
|---|---|
| `Teltonika: <hwid>: connection` | OK/ERROR/STALE as above |
| `Teltonika: <hwid>: system` | OK from cached `system/board` dict |
| `Teltonika: <hwid>: cellular` | Existing cellular state mapping, driven from cache |
| `Teltonika: <hwid>: mwan3/<name>` | Per-mwan3-member dynamic set — see §3 |
| `Teltonika: <hwid>: interface/<name>` | Per-interface dynamic set — see §3 |

### 3. Dynamic membership — `Updater.add` / `Updater.removeByName`

MikroTik interfaces, wireless registrations, and Teltonika mwan3 members
have variable membership (interfaces renamed, wireless clients come and
go, mwan3 members added). The plain Starlink pattern (fixed six tasks)
doesn't apply. Approach:

- After each successful `poll_callback`, compare the observed name set
  against `self._registered_dynamic_tasks` (a set of the names we've
  already `Updater.add`-ed).
- For names in `observed − registered`: call `updater.add(name, cb)` —
  `cb` is a per-entity closure that reads from the cache.
- For names in `registered − observed`: call `updater.removeByName(name)`
  and drop from the set. **But:** give a grace period (e.g., the first
  poll where the name is missing just marks the cache entry stale
  instead of removing; only remove after `max_data_age * 2` without
  reappearing). This prevents churn on transient flaps.
- Dynamic mutation happens on the rclpy thread via the Updater's own
  `update()` invocation cadence — callbacks into `updater.add/removeByName`
  are serialized against task callbacks, so this is safe without extra
  locking beyond the cache lock.

Skip this section's complexity for **ping_monitor**: target membership
is static.

### 4. Extract pure logic into testable modules (mirror
`starlink_stats.diagnostics_logic`)

For each monitor, pull the "cache snapshot → DiagnosticStatus fields"
synthesis into a pure module named **`diagnostics_logic.py`** (same
name in each package, matching Starlink's convention for fleet-wide
consistency). The module takes primitives (dicts, ints, `now_monotonic`)
and returns `(level, message, kvs)`. The node keeps ROS integration;
the logic module gets comprehensive unit tests covering:

- OK / WARN / ERROR thresholds
- STALE emission when cache is None or too old
- The specific schema-drift scenario from this issue (verify that after
  an error → success transition, task names are unchanged)

### 5. Preserve `last_query_time` KeyValue convention

PR #14 added `last_query_time` as an ISO-8601 KeyValue on every status.
Keep it: `_apply_common_kv` reads `cache.poll_wall_iso` and appends
`KeyValue(key='last_query_time', value=cache.poll_wall_iso or 'never')`.

### 6. Align timing parameters with Starlink

Two renames, driven by the Updater migration:

| Old param (all three nodes) | New param | Default | Semantics |
|---|---|---|---|
| `max_data_age_s` | `stale_timeout_sec` | 5.0 | Cache-age threshold above which tasks emit STALE |
| `publish_interval` | `update_period_sec` | 1.0 | `diagnostic_updater.Updater` publish cadence |

**`poll_interval` stays as-is.** An earlier draft proposed renaming it
to `poll_interval_sec` for `_sec`-suffix consistency, but (a) Starlink
uses `poll_period_sec` anyway so the consistency argument doesn't hold,
and (b) `poll_interval` is set externally in three in-repo config YAMLs
(`ping_targets.yaml`, `test_ping_targets.yaml`, `teltonika_monitor.yaml`)
— renaming it cascades into config churn for no behavioral benefit.

**No deprecated-alias path for the two renames.** A workspace grep
confirmed no launch files or YAML configs anywhere in the workspace
pass `max_data_age_s` or `publish_interval` externally — every reference
is inside the three monitor nodes' own `declare_parameter` calls.
Rename cleanly and document the rename in the PR body.

### 7. Consider `rqt_operator_tools#14` incidentally

That issue is about the test config `config/test_ping_annunciator.yaml`
expecting `ping.test: localhost` when the node publishes `Ping: localhost`.
Root cause is `hardware_id=''` at launch so the name prefix degrades to
just `Ping`. Fix is either in the launch file (pass `hardware_id='ping.test'`)
or in the config file (expect `Ping: localhost` and drop the `ping.test` prefix).

This isn't caused by the Updater migration and isn't fixed by it. Leave it
out of this PR — it's a one-line launch-file or config-file edit tracked
on a separate issue.

## Files to Change

### Nodes + pure-logic modules + tests

| File | Change |
|---|---|
| `network_tools/network_tools/ping_monitor_node.py` | Replace `poll_callback`/`_publish_callback` with Updater pattern. Drop `_cache_lock` manual publish scaffolding. |
| `network_tools/network_tools/diagnostics_logic.py` | **New** — pure level/message synthesis for ping samples. |
| `mikrotik_monitor/mikrotik_monitor/mikrotik_monitor_node.py` | Replace with Updater pattern; add `: connection` task; dynamic interface/wireless membership management. Delete the name-dropping error path. |
| `mikrotik_monitor/mikrotik_monitor/diagnostics_logic.py` | **New** — pure synthesis helpers. |
| `teltonika_monitor/teltonika_monitor/teltonika_monitor_node.py` | Same pattern as Mikrotik; add `: connection` task; dynamic interface/mwan3 membership. |
| `teltonika_monitor/teltonika_monitor/diagnostics_logic.py` | **New** — pure synthesis helpers. |
| `network_tools/test/test_diagnostics_logic.py` | **New** — unit tests for ping logic. |
| `mikrotik_monitor/test/test_diagnostics_logic.py` | **New** — unit tests including schema-drift regression. |
| `teltonika_monitor/test/test_diagnostics_logic.py` | **New** — unit tests including schema-drift regression. |

### Packaging

| File | Change |
|---|---|
| `network_tools/package.xml` | Add `<depend>diagnostic_updater</depend>`. |
| `mikrotik_monitor/package.xml` | Add `<depend>diagnostic_updater</depend>`. |
| `teltonika_monitor/package.xml` | Add `<depend>diagnostic_updater</depend>`. |

### Configs (in-repo, must follow the param renames)

| File | Change |
|---|---|
| `network_tools/config/ping_targets.yaml` | Verify — currently sets `poll_interval` (unchanged), no renamed params. Likely no edit. |
| `network_tools/config/test_ping_targets.yaml` | Verify — currently sets `poll_interval`, `hardware_id` (both unchanged). Likely no edit. |
| `teltonika_monitor/config/teltonika_monitor.yaml` | Add `stale_timeout_sec`, `update_period_sec` with explicit defaults so field deployments don't rely on node-side defaults. |

### Launch files (verify unchanged)

| File | Change |
|---|---|
| `network_tools/launch/ping_monitor.launch.py` | Verify — does not currently set renamed params. Likely no edit. |
| `mikrotik_monitor/launch/mikrotik_monitor.launch.py` | Verify — does not currently set renamed params. Likely no edit. |
| `teltonika_monitor/launch/teltonika_monitor.launch.py` | Verify — does not currently set renamed params. Likely no edit. |

### Docs

| File | Change |
|---|---|
| `README.md` | Short note: nodes use `diagnostic_updater.Updater`; `: connection` task now surfaces reachability instead of a summary-named status. |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| **A change includes its consequences** | Plan includes test migration, `package.xml` deps, README note about the `: connection` task name change. Not just the node code. |
| **Only what's needed** | Refactor is scoped to the Updater migration; doesn't rewrite clients, swap libraries, or expand feature set. `rqt_operator_tools#14` deliberately excluded. |
| **Improve incrementally** | Three monitors migrate in the same PR because they share the new helper shape and should land together to keep the pattern consistent. Each monitor's change is independently readable; commits kept per-monitor for reviewability. |
| **Test what breaks** | Regression tests specifically cover the error → success schema-drift scenario. Pure-logic tests (no rclpy context) mirror Starlink's `diagnostics_logic` coverage depth. |
| **Workspace vs project separation** | N/A — this is work inside a project repo. |
| **Human control and transparency** | Param name changes (`max_data_age_s` → `stale_timeout_sec`, `publish_interval` → `update_period_sec`) flagged in PR body. Clean rename with no aliases because workspace grep confirmed no external callers. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| **0002 — Worktree isolation** | Yes | Work in `layers/worktrees/issue-ros2_network_monitor-15`, branch `feature/issue-15`. |
| **0008 — Follow ROS 2 Official Conventions** | Yes | `diagnostic_updater.Updater` *is* the ROS 2 convention for exactly this problem. Moving to it is compliance, not deviation. |
| 0003 — Project-agnostic workspace | No | Changes are all inside a project repo. |
| 0009 — Python package management | No | No new Python deps beyond `diagnostic_updater`, which is available via rosdep. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Diagnostic task names (`MikroTik: <hwid>` → `: connection`) | Any annunciator config that exact-matched the old summary name | Workspace grep (2026-04-23) confirmed **no such configs exist** anywhere in `layers/main/`. Consumers use `startswith` or per-task-suffix matching. Strictly additive change; no consumer migration needed. |
| Parameter names (`max_data_age_s` → `stale_timeout_sec`; `publish_interval` → `update_period_sec`) | Any launch file or YAML config passing the old names | Workspace grep confirmed no external callers. Rename cleanly; document in PR body. |
| `package.xml` deps (add `diagnostic_updater`) | `rosdep` database on deployment machines (pre-existing dep — already installed for Starlink) | No separate deploy step needed. |
| Extract pure logic into `*_logic.py` modules | Imports in any external code using internal monitor modules | None known — modules are currently all internal. |

## Open Questions

1. ~~**Deprecated-alias behavior for old params.**~~ **Resolved.**
   Workspace grep (2026-04-23) across `layers/main/**/*.{py,yaml,yml,launch*,xml}`
   found zero external callers of `max_data_age_s` or `publish_interval` —
   every reference is inside the three monitor nodes' own
   `declare_parameter` calls. No alias path needed.
2. **Dynamic-membership grace period.** How many missed polls before
   `removeByName` is called on a disappeared interface? Proposal above
   says `stale_timeout_sec * 2`. Reasonable without field data, but the
   first field session with the new nodes should explicitly observe
   transient-flap frequency (e.g., Teltonika interfaces cycling during
   mwan3 handoffs) and tune the default. Tunable meanwhile via a
   `dynamic_task_grace_sec` param. **Follow-up to open as a field
   observation task once the PR merges.**
3. **One PR or three?** Leaning "one PR, three commits" — the helper
   extraction is shared enough that splitting three ways duplicates
   review. But if Claude Code loses signal on PR size during review,
   the three-PR split is viable. Defaulting to one PR; revisit if the
   diff grows past ~800 LOC changed.
4. **Project-level governance.** `layers/main/sensors_ws/src/ros2_network_monitor/`
   has no `.agents/README.md`. Not required for this PR, but the gap
   should be called out as a follow-up issue at some point.

## Estimated Scope

Single PR, three commits (one per monitor), plus a prep commit extracting
the shared `CachedStatus`/`_apply_common_kv` scaffolding if the duplication
warrants it. Target diff: ~600–800 LOC net (new test code is most of it;
node code should shrink modestly). Estimated effort: 1–2 focused sessions.
