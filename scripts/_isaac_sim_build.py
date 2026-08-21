"""Check whether this installed package was built with Isaac Sim configured."""

from __future__ import annotations

from pathlib import Path
import re
import sys


def require_isaac_sim_build(script_path: str) -> None:
    """Exit with an actionable error unless the installed config has a path."""
    # Do not resolve this path: under --symlink-install that would follow the
    # script link back into src, while the generated config belongs in build.
    config = Path(script_path).absolute().parents[1] / "share" / "isaac_sim.yaml"
    value = None
    if config.is_file():
        match = re.search(
            r'^isaac_sim_dir:\s*["\']?([^"\'\s]+)',
            config.read_text(encoding="utf-8"),
            re.MULTILINE,
        )
        if match:
            value = match.group(1)
    if value not in {None, "null", "None"}:
        return
    print(
        "error: Isaac Sim is not configured. Build the workspace with "
        "ISAAC_SIM_DIR set to the Isaac Sim root directory.",
        file=sys.stderr,
    )
    raise SystemExit(2)


if __name__ == "__main__":
    require_isaac_sim_build(__file__)
