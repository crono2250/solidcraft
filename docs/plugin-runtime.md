# Experimental command plugin runtime

This is the implemented one-shot native runtime. It does not implement every part of
[issue #1221](https://github.com/tangency-labs/solidcraft/issues/1221), the broader
[interface proposal #4980](https://github.com/tangency-labs/solidcraft/pull/4980), or
[Utilities placement proposal #4933](https://github.com/tangency-labs/solidcraft/pull/4933).
The engine/log/store/command interface names and command world follow #4980; the initial package
manifest uses an optional `runtime` block so old metadata-only packages still work. Its manifest
layout and explicit per-invocation approval flow are deliberately documented as experimental,
not as a claim that the outstanding design proposals have been accepted.

## Boundary and lifecycle

1. Put an independently built component beside a static `plugin.json` in its id folder.
2. `plugin.inspect` reads bounded bytes through the caller's existing file scope and returns
   the manifest and content approval hash. It neither instantiates the component nor invokes
   a manifest function. Metadata can therefore be reviewed before code runs.
3. The user reviews the package and explicitly calls `plugin.run` with that exact hash, an
   exported entry and JSON parameters. The package is read once for this call; the bytes whose
   hash was checked are the bytes compiled and executed. Changing the files afterward cannot
   substitute different code for that invocation.
4. The host creates a fresh Wasmtime engine/store, with fuel, epoch interruption and a resource
   limiter, and links only the published component interfaces. No WASI is linked. Instantiation
   and the entry share the same execution budget, including component start functions.
5. `engine.run` reaches `Session::execute` on a scratch session, under an empty file scope.
   Explicit paths and implicit reads during feature evaluation cannot inherit the caller's broader
   file access. Results are bounded JSON. Refused calls poison the transaction, even if guest code
   catches the error and returns success.
6. The result must be valid JSON, every newly changed feature must evaluate, and the call must
   remain within its budget. Only then are the native results accepted as one undo step.
7. The desktop control channel performs the whole computation on a worker, warms the resulting
   display and adopts the existing computed-commands transaction exactly once. A changed design,
   authoring context, selection, disabled-plugin policy, or Stop prevents adoption. It does not
   re-run the plugin on the window thread. Headless CLI execution is synchronous.

No discovery, preference toggle, document open or saved feature executes a component. Every
invocation gets a fresh instance, so guest memory is not persistent state. A design is data, never
an instruction to install, approve, download or execute a package.

## Manifest

The metadata fields remain `id`, `name`, `version`, and optional `description`. IDs are 1..64
lowercase letters, digits and hyphens, without a leading/trailing hyphen, and must equal the folder
name. Names are at most 80 characters; versions at most 32; descriptions at most 240. Control
characters and unknown fields are refused. The manifest has a 16 KiB byte cap.

`runtime` is optional for metadata-only packages. Executable packages declare:

| Field | Meaning |
|---|---|
| `api` | Exactly `solidcraft:plugin@0.1.0`; 0.x is not a stable API promise |
| `module` | One lowercase `.wasm` file name within the package; no directories |
| `entries` | 1..32 distinct entry ids passed to `command.execute` |
| `commands` | At most 128 exact existing built-in command ids, intersected with host policy |
| `limits` | Optional object with all of `fuel`, `memory_mib`, `deadline_ms`; omitted object uses defaults |

The component must be a WebAssembly **component**, not an ordinary core `.wasm` module, DLL,
executable or script. Unsupported imports fail instantiation; the host never falls back to starting
another process. A dependency is not installed when a component fails to load.

## Command and data surface

The host admits existing `solid.*` and `sketch.*` commands (except file-based `sketch.insert_*`),
plus `parameters.add/change/delete`, `timeline.add/inspect/redefine/delete`, `document.inspect`,
`model.faces`, `model.edges`, `model.mesh` and `inspect.measure`. Admission still requires the exact
id in the package manifest. New families require a host policy review. Wildcards, command aliases,
plugin-to-plugin calls, batches/scripts and `file.*`/`doc.*` commands are not guest capabilities.
The empty file scope also applies to a feature that names an external font or linked design.

`model.mesh {body, tolerance?}` is a general read-only input boundary. Tolerance is a finite positive
number in millimetres (default 0.05). It returns `body`, its owning `component`,
`frame: "component-local-mm"`, `positions: [[x,y,z], ...]` and `triangles: [[i,j,k], ...]`. Indices are
zero-based. The mesh is of the evaluated body, rather than only the original import's vertices.
The command refuses more than 8192 vertices or 16384 triangles. A plugin's host-call result must
also fit the JSON byte limit; a smaller mesh can still exceed that limit and be refused.
No reconstruction, fitting, sewing, mesh replacement or conversion algorithm is part of this API.

A plugin computes its specialised behaviour externally, then emits ordinary native commands or
validated native features through `timeline.add`. New algorithms do not acquire private kernel
entry points. This initial boundary does not promise arbitrary B-rep construction, a stable binary
kernel serialization, persistent custom feature types, or automatically replaying plugin code on
parameter changes. A later specialised plugin must prove its own geometry, failure, lineage,
attachment, undo and reopen contracts in its own change.

## Consent and identity

`plugin.inspect` returns SHA-256 of a domain tag, the manifest byte length, exact manifest bytes,
and exact component bytes. `plugin.run` compares it before instantiation. Whitespace changes to a
manifest also require reinspection. A matching digest is an identity check, not a signature;
reviewing permissions does not establish whether a publisher is trustworthy.

There is no persistent automatic approval list in this version. A caller supplies the reviewed
content identity on every invocation. Agent chat separately asks before running external code;
its model cannot turn an inspected hash into user consent. A host or MCP caller is responsible
for its existing user-consent policy and package-folder scope. Desktop disable preferences apply
to the running desktop session and all plugin worker copies; a separate CLI process has its own
policy, not the GUI's preferences.

## Budgets and limits

| Resource | Default / hard maximum |
|---|---|
| Component input | 16 MiB |
| Guest fuel per invocation | 10,000,000 / 100,000,000,000 |
| Guest memory | 64 MiB / 128 MiB; one memory total |
| Entry deadline | 5 s / 30 s |
| Core instances / tables | 8 / 2 (guest plus canonical ABI shim) |
| Table elements / guest stack | 10,000 / 512 KiB |
| Host engine calls | 256 |
| Params, one host result, final result | 1 MiB each, bounded JSON parser depth |
| Log messages | 1 KiB each, 64 KiB per call; control characters stripped |

The default fuel remains 10 million. Larger budgets require an explicit manifest value and a new
content approval; the host never raises a budget or retries automatically. Fuel counts metered
Wasm operations rather than milliseconds. A substantial separately compiled CAD routine can use
billions of fuel units; the independent 30-second deadline and 128 MiB memory ceiling still apply
to an explicitly approved larger request.

Fuel and epoch checks contain guest execution, not arbitrary native work. Component compilation
happens before the guest deadline; its input size is bounded, but compiler memory/time are not
covered by guest fuel. Each approved native command runs under its existing engine/kernel limits;
the guest deadline prevents more commands rather than forcibly interrupting a native call.
The desktop worker keeps this work off the window thread. Stop drops adoption; an already running
worker finishes unseen. It does not synchronously kill a native compiler or kernel call.

Execution errors report the root cause before the Wasm backtrace, the typed trap (for example
`OutOfFuel`, `Interrupt`, `MemoryOutOfBounds` or `UnreachableCodeReached`), consumed and remaining
fuel, and the configured fuel, memory and deadline limits. A host/linker/resource error can have
`trap=none`; its complete cause chain is still reported, including a refused memory/table growth.
`Interrupt` identifies epoch interruption; it is distinct from fuel exhaustion. These diagnostics
do not automatically retry a component, raise its limits or adopt a failed transaction. Successful
invocations also write consumed/remaining fuel to the existing debug logger. In the CLI, pass
`--debug` to see these counters on stderr; normal command output and plugin result JSON stay the
same. No model contents or package paths are included in that counter message.

There is no compiled-code cache, unsafe deserialization, WASI filesystem/clock/socket/process API,
network integration, persistent store, plugin installer, signed index, custom panels or icons.
The WIT store interface is present for command-world compatibility, but operations are denied;
`previous-version` returns none. `manifest()` is not used.

## Author an original plugin

See [the Rust box example](../examples/plugins/example-box/README.md). It is intentionally a small
native-command example, not a bundled production feature. The host compiles no plugin algorithm
into SolidCraft. Rust first builds a core module with WIT metadata; `wasm-tools component new`
wraps it as a component without a WASI adapter. Other languages may use the same published
[WIT](../wit/solidcraft-plugin.wit) if they produce compatible components without unsupported imports.

## Platforms and validation

Native execution on x86-64 or AArch64 requires the engine's `plugins` Cargo feature.
Other native architectures retain the commands but report that no execution host is available. The browser build publishes the
same command descriptions but refuses execution without a native host. Turning off this feature
keeps metadata, inspection and the general mesh query available while excluding Wasmtime.
No new design format or custom `FeatureKind::Plugin` is introduced.

The runtime tests use original generated components, not mocks of a successful plugin: host calls,
two edits grouped into one undo step, measured volume, undo/redo and cold reopen after removing
the package; guest traps, fuel/memory exhaustion, forbidden calls, stale approval, disabled plugins,
and caller path scopes. The desktop tests cover worker scheduling and rejection of stale results.
The PR records actual validation results; this guide is not a claim that a particular build passed.
