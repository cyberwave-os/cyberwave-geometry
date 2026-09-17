# Golden vectors

`geometry_golden.json` is the cross-language conformance suite for the geometry
core. One file, read by every binding's test runner, so "C++ and TypeScript
agree" is something the CI proves rather than something we hope.

## Format

```jsonc
{
  "version":   "0.1.0",        // core version that generated it
  "tolerance": 1e-12,          // absolute comparison budget
  "robots":    { "<name>": { "links": [], "joints": [], "sensors": [] } },
  "cases": [
    {
      "id":     "quat.normalize.general",
      "op":     "quat_normalize",
      "input":  { "q": { "w": 1, "x": 0, "y": 0, "z": 0 } },
      "expect": { "status": "ok", "quaternion": { "w": 1, "x": 0, "y": 0, "z": 0 } }
    }
  ]
}
```

* Quaternions are objects with named `w`/`x`/`y`/`z`; vectors with named
  `x`/`y`/`z`. Never positional — the whole library exists because a bare
  four-element array is ambiguous between `xyzw` and `wxyz`.
* Matrices are row-major nine-element arrays.
* `status` is `"ok"` or an error name from `CONVENTIONS.md` §5.
* `fk_*` cases reference a description by name from the top-level `robots` map.

## Writing a conformance runner

Implement the `op` dispatch — the reference is `golden::evaluate` in
[`../tests/golden_ops.cpp`](../tests/golden_ops.cpp), and a worked
implementation in another language is
[`../bindings/python/tests/test_golden.py`](../bindings/python/tests/test_golden.py).
For each case: run `op` on `input`, compare the result with `expect` field by
field, numbers within `tolerance`.

## Deliberately absent: NaN and infinity

JSON has no portable spelling for them, and encoding them as magic strings
would put a parser quirk in four languages to test one behaviour. Non-finite
input is covered by each language's own unit tests instead — `test_quaternion.cpp`,
`bindings/python/tests/test_binding.py`.

## Regenerating

```bash
cmake --build build/geometry --target golden_gen
build/geometry/tests/golden_gen --write common/geometry/golden/geometry_golden.json
```

Only after an *intentional* behaviour change, and read the diff before
committing it.
