#!/usr/bin/env python3

from __future__ import annotations

import signal
import subprocess
from pathlib import Path


def main() -> int:
    child = subprocess.Popen([str(Path.cwd() / ".venv" / "bin" / "dora-rerun")])
    try:
        return_code = child.wait()
    except KeyboardInterrupt:
        child.terminate()
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
        return 0

    if return_code in {-signal.SIGTERM, -signal.SIGINT}:
        return 0
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
