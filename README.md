# <DOBOT-CT>

Control and experimentation framework for a synchrotron X-ray CT sample changer operated by a DOBOT robot via TCP/IP communication.

This repository is intended primarily for research and internal development use.

It is under active development and is NOT a packaged or production-ready distribution.
The internal structure, APIs, and behavior may change without notice.
Use at your own risk.

---

## External Dependencies

This project currently depends on third-party APIs.

### DOBOT TCP-IP Python API

At present, the project uses the official DOBOT TCP-IP Python API (V4).

For installation instructions, see:

    dobot/README.md

This repository does NOT redistribute any third-party proprietary code.
Users must obtain required external components directly from their official sources.

---

## Development Status

- Research / experimental use
- Subject to change without backward compatibility guarantees
- No stability guarantees

---

## Disclaimer

This project is provided "as is", without warranty of any kind.
The author assumes no responsibility for hardware damage,
data loss, or unintended robot motion.

Always verify motion commands in a safe environment before real operation.