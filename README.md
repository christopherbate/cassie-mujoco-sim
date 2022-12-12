This is a rebuild of the original cassie-mujoco-sim project.

The major improvements focus on:
- CMake build system
- Python packaging
- Cleanup of C code
- C sanitizer support (asan, etc.)

[Original docs](docs/README.md).

### Build

Requires:
- cmake (recent, tested with 3.25)

Mujoco 2.3 install (with dev headers).

Python requirements for building:

```
pip install ctypeslib2
```

```
make configure
make build
```

Build python package:

```
make wheel
```
