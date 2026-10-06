# Security Policy

## Scope

mtilt reads a decad body, a JSON profile and options, and adds bodies to the body's decad document. It reads and
writes no files of its own. The primary security concern is **crafted input**: a profile, option set or body that
reaches the public API and causes a denial of service (unbounded CPU or memory, a hang) or a reachable panic. Bugs
of that kind are in scope.

Issues that only affect test code, examples, or internal tooling are welcome as ordinary bug reports rather than
security reports.

## Supported Versions

No release is tagged yet. `main` is the reference and only line that receives fixes.

| Version  | Supported                    |
| -------- | ---------------------------- |
| v0.x.x   | :white_check_mark: (pre-1.0) |
| < v0.1.0 | :x: (unreleased)             |

## Reporting a Vulnerability

If you think you found a vulnerability, please report it via
[GitHub Security Advisory](https://github.com/lestrrat-3d/mtilt/security/advisories/new).
Please include explicit steps to reproduce the security issue — a minimal reproducer or failing test, and the
commit SHA you tested against, are ideal.

We will do our best to respond in a timely manner, but please also be aware that this project is maintained by a
very limited number of people. Please help us with test code and such.
