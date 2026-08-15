"""Locate and import the OpenVSP 3.51.2 Python API without installing it.

The OpenVSP release archive ships a Python source tree whose ``setup.py`` declares
sibling local packages as requirements and whose ``MANIFEST.in`` omits the Windows
``_vsp.pyd`` extension, so ``pip install`` silently produces a broken package (see
``docs/openvsp_real_api_findings.md``). Importing straight off the extracted bundle
sidesteps both problems: the ``.pyd`` is already sitting next to ``vsp.py``.

Override the search with ``DOUGLAS_DART_OPENVSP_ROOT`` (the directory containing
``vsp.exe``).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from types import ModuleType

EXPECTED_VERSION = "3.51.2"

# Where a 3.51.2 Windows bundle is normally extracted on this machine. The
# environment variable wins; these are only the fallbacks.
_CANDIDATE_ROOTS = (
    r"C:\Users\dougl\Downloads\OpenVSP-3.51.2-win64-Python3.11\OpenVSP-3.51.2-win64",
    r"C:\Users\dougl\Documents\OpenVSP\OpenVSP-3.51.2-win64",
)

_SUBPACKAGES = ("openvsp", "degen_geom", "utilities", "openvsp_config")


def find_openvsp_root() -> Path:
    """Return the directory holding ``vsp.exe`` and the ``python/`` bundle."""

    override = os.environ.get("DOUGLAS_DART_OPENVSP_ROOT")
    candidates = ([override] if override else []) + list(_CANDIDATE_ROOTS)
    for candidate in candidates:
        if not candidate:
            continue
        root = Path(candidate)
        if (root / "vsp.exe").is_file() and (root / "python").is_dir():
            return root
    raise RuntimeError(
        "No OpenVSP "
        + EXPECTED_VERSION
        + " bundle found. Set DOUGLAS_DART_OPENVSP_ROOT to the directory that "
        "contains vsp.exe and python/. Searched: " + ", ".join(str(c) for c in candidates)
    )


def vspaero_executable(root: Path | None = None) -> Path:
    """Return the bundled ``vspaero.exe``.

    VSPAERO is a separate solver binary that the API shells out to. The API finds
    it by searching PATH, which is why :func:`load_openvsp` prepends the bundle.
    """

    root = root or find_openvsp_root()
    exe = root / "vspaero.exe"
    if not exe.is_file():
        raise RuntimeError(f"vspaero.exe is missing from the OpenVSP bundle: {root}")
    return exe


def load_openvsp() -> ModuleType:
    """Import ``openvsp`` off the extracted bundle and check its version.

    Also prepends the bundle root to ``PATH`` so the API can find ``vspaero.exe``
    and the Windows DLL loader can resolve the compiled extension's dependencies.
    """

    root = find_openvsp_root()
    python_bundle = root / "python"
    for package in _SUBPACKAGES:
        package_dir = python_bundle / package
        if package_dir.is_dir():
            path_entry = str(package_dir)
            if path_entry not in sys.path:
                sys.path.insert(0, path_entry)

    os.environ["PATH"] = str(root) + os.pathsep + os.environ.get("PATH", "")
    # Python 3.8+ no longer honours PATH for extension-module DLL resolution.
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(root))

    import openvsp  # noqa: PLC0415  -- deliberately late, after sys.path is set

    actual = str(openvsp.GetVSPVersion())
    if EXPECTED_VERSION not in actual:
        raise RuntimeError(
            f"OpenVSP version mismatch: expected {EXPECTED_VERSION}, got {actual!r} "
            f"from {openvsp.__file__}"
        )
    if not hasattr(openvsp, "AddGeom") or not callable(openvsp.AddGeom):
        raise RuntimeError(
            f"{openvsp.__file__} imports but is not a functional OpenVSP API "
            "(AddGeom missing). A namespace-only 'openvsp' is probably shadowing it."
        )
    return openvsp


def _error_manager(vsp: ModuleType):
    """Return the API's error manager.

    NOTE, and it matters: this build does NOT expose ``GetNumTotalErrors`` /
    ``PopLastError`` at module level -- only ``ErrorMgrSingleton``. Any error
    check written as ``if hasattr(vsp, "GetNumTotalErrors")`` therefore never
    fires here, which is exactly how a run full of "Can't Find Parm" errors can
    report success. Verified against the installed 3.51.2 Windows build.
    """

    manager = getattr(vsp, "ErrorMgrSingleton", None)
    if manager is None:
        raise RuntimeError(
            "The imported openvsp module exposes no ErrorMgrSingleton, so API "
            "errors cannot be detected. Refusing to build blind."
        )
    return manager.getInstance()


def clear_errors(vsp: ModuleType) -> None:
    """Drop anything already on the stack so a stale error blames the wrong step."""

    manager = _error_manager(vsp)
    while manager.GetNumTotalErrors() > 0:
        manager.PopLastError()


def check_errors(vsp: ModuleType, context: str = "") -> None:
    """Drain OpenVSP's error stack and raise if anything is on it.

    The API reports most failures by pushing onto this stack and returning a
    plausible-looking value, so a silent build is not evidence of a good build.
    """

    manager = _error_manager(vsp)
    errors: list[str] = []
    while manager.GetNumTotalErrors() > 0:
        error = manager.PopLastError()
        errors.append(
            str(error.GetErrorString()) if hasattr(error, "GetErrorString") else str(error)
        )
    if errors:
        where = f" during {context}" if context else ""
        # Newest first, and de-duplicated: one bad parm name typically pushes the
        # same pair of messages once per geom.
        unique = list(dict.fromkeys(errors))
        raise RuntimeError(
            f"OpenVSP reported {len(errors)} API error(s){where}:\n  - "
            + "\n  - ".join(unique)
        )
