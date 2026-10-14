# Runtime provenance

Source: https://github.com/LIG-Embedded-School-Team-2-WarWarden/c2

Imported from the committed snapshot
`d9b787e51f2764a641e8d393a21a0c1433167c6f`. Uncommitted changes were excluded:
they require unpublished protocol extensions and would break the pinned wire contract.
The original checkout has not been modified. The MIT notice is preserved in LICENSE.
This snapshot is owned and built by this repository; it does not depend on a sibling checkout.

The shared wire contract is a separate, unmodified git submodule:
https://github.com/LIG-Embedded-School-Team-2-WarWarden/protobuf
at `36be2707f6b88769118a0fb6519a8c5193e0ff3b`.

`include/c2/control_center.hpp` at the repository root is the earlier simulation exercise.
The live runtime uses `runtime/include/c2/` and the real protocol types. These two APIs are
intentionally built as separate targets, never included together in one translation unit.
