# Nix

## Base syntax

Prefer 'set patterns' over named attribute sets in function declarations for all
functions except overlays:

```{nix}
{x, y, z}: x + y + z;
```

rather than:

```{nix}
foo: foo.x + foo.y + foo.z;
```

For overlays, use `prev` and `final` as arguments over `self` and `super`. For
overlays which are not top-level (i.e. to overlay a set contained within
`pkgs`), use similar naming (e.g. for a ROS overlay, use `rosPrev` and
`rosFinal`).

Use string interpolation over string concatenation where possible:

```{nix}
foo = "Hello";
bar = "${foo} World";
```

over:

```{nix}
foo = "Hello";
bar = foo + " World";
```

## Flakes

### Inputs

Ensure there are no duplicate inputs (e.g. a dependency pulls in something that
another input also pulls in) so less downloads are needed. If there are
differences in versioning, use the newer version of the dependency. The
exception to this is if two versions of an input are desired (e.g. nixpkgs-25.11
and nixpkgs-unstable).

### Outputs

ROAR uses two architectures: x86_64-linux and aarch64-linux. For anything that
is to be run at competition, both of these should be used in the outputs of the
flake. For anything to be run only during development, only x86_64-linux is
needed. When in doubt (or for simplicity), use the
[flake-utils](https://github.com/numtide/flake-utils) `eachDefaultSystem`.

## Formatting

All nix code should be formatted with [nixfmt](https://github.com/NixOS/nixfmt).
