# CANopenNode (vendored)

A copy of the protocol code from [CANopenNode](https://github.com/CANopenNode/CANopenNode), Apache-2.0 (see `LICENSE`).

- **Upstream:** `v4.1-9-g9b8beed`, commit `9b8beed8367241e96ac03f916cd5a500bcb2cf23` (2026-07-10).
- **Copied:** `CANopen.c/h`, `301/`, `storage/CO_storage.c/h`, and the headers of `303/`, `304/`, `305/`, `309/` and `extra/` (`CANopen.h` includes them; their features are off in `CANopen/port/CO_driver_target.h`).
- **Left out:** `301/CO_SDOclient.c`, `CO_TIME.c` and `CO_Node_Guarding.c` (disabled), the examples, docs and tests.
- **Not modified.** The configuration lives in `CANopen/port/CO_driver_target.h`. The object dictionary (`CANopen/od/OD.c/h`) is exported by CANopenEditor from `driverTCC.xdd`.

To update: copy the same files from a newer tag over these, rebuild, and note the new commit here.
