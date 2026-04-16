# Local TARE Patches

Log every non-trivial deviation from the vendored upstream snapshot so
the next rebase is painless.

## Vendored snapshot

    upstream: caochao39/tare_planner
    branch:   humble-jazzy
    imported: 2026-04-16

The vendored tree omits these upstream directories because they are not
needed in production and would bloat the LingTu repo:

  - `or-tools/`    — 130+ MB x86-64 binary (fetched separately by build script)
  - `rviz/`        — RViz config (visualization handled by LingTu Rerun stack)
  - `data/`        — example pcds

## Modifications

### CMakeLists.txt
Removed two `install(DIRECTORY ...)` rules for `rviz/` and `data/` so the
build does not fail with "directory not found" after we dropped those
upstream folders.

