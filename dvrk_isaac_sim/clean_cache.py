"""Remove the generated Isaac Sim asset cache for the current workspace."""

from __future__ import annotations

import shutil
from pathlib import Path


def _workspace_cache_from_cwd() -> Path | None:
    current = Path.cwd().resolve()
    for candidate in (current, *current.parents):
        if (candidate / "src").is_dir():
            cache_candidates = (
                candidate / ".generated" / "isaacsim-6.0",
                candidate / "build" / "dvrk_isaac_sim" / "share" /
                ".generated" / "isaacsim-6.0",
                candidate / "install" / "dvrk_isaac_sim" / "share" /
                ".generated" / "isaacsim-6.0",
            )
            for cache in cache_candidates:
                if cache.is_dir() or cache.parent.is_dir():
                    return cache
            return cache_candidates[0]
    return None


def _workspace_cache_from_ament() -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("dvrk_isaac_sim")).resolve()
    except (ImportError, LookupError, RuntimeError):
        return None

    # Development and installed layouts place the fallback cache next to the
    # package's share directory, e.g. build/.../share/.generated.
    share_cache = share.parent / ".generated" / "isaacsim-6.0"
    if share_cache.is_dir():
        return share_cache

    for parent in share.parents:
        if parent.name in {"build", "install"}:
            return share.parent / ".generated" / "isaacsim-6.0"
    return None


def _cache_path() -> Path | None:
    return _workspace_cache_from_cwd() or _workspace_cache_from_ament()


def main() -> int:
    cache = _cache_path()
    if cache is None:
        print("Unable to identify a colcon workspace from the current directory.")
        return 1

    if cache.name != "isaacsim-6.0" or cache.parent.name != ".generated":
        print(f"Refusing unsafe cache path: {cache}")
        return 1
    if cache.is_symlink():
        print(f"Refusing to remove symlink: {cache}")
        return 1
    cache = cache.resolve()
    if not cache.exists():
        print(f"Cache does not exist: {cache}")
        return 0

    entries = sorted(path.name for path in cache.iterdir())
    if entries:
        print("Configurations found:")
        for entry in entries:
            print(f"  - {entry}")
    else:
        print("Configurations found: none")

    shutil.rmtree(cache)
    print(f"Removed generated Isaac Sim cache: {cache}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
