import importlib
import sys
import os
import sys
import site


def blue() -> str:
    in_venv: bool = sys.prefix != sys.base_prefix
    venv_path: str = sys.prefix if in_venv else "None detected"

    return f"""\

Outside of venv

Current Python: {sys.executable}
Virtual Environment: {venv_path}

Warning: You're in the global environment!

run:
python -m venv fly_in_venv
source fly_in_venv/bin/activate # On Unix
fly_in_venv\\Scripts\\activate # On Windows

Then run this program again"""


venv_name = os.path.basename(sys.prefix)
package_path = site.getsitepackages()[0]


def red() -> str:
    return f"""\

Welcome

Curremt Python: {sys.executable}
Virtual Environment: {venv_name}
Environment: Path: {sys.prefix}

SUCCESS: You're in an isolated environment!

Package installation path:
{package_path}"""


def in_venv() -> bool:
    return hasattr(sys, 'real_prefix') or sys.base_prefix != sys.prefix


REQUIRED = {
    "pandas": "Data manipulation ready",
    "numpy": "Numerical computation ready",
    "matplotlib": "Visualization ready",
}


def check_dependencies():
    print("Checking dependencies:")
    missing = []
    for name, desc in REQUIRED.items():
        try:
            module = importlib.import_module(name)
            version = getattr(module, "__version__", "unknown")
            print(f"[OK] {name} ({version}) - {desc}")
        except ImportError:
            print(f"[MISSING] {name} - {desc}")
            missing.append(name)
    return missing


def print_install_instructions(missing):
    print("\nMissing dependencies:", ", ".join(missing))
    print("\nInstall with pip:")
    print("  pip install -r requirements.txt")
    print("\nInstall with Poetry:")
    print("  poetry install")
    print("  poetry run python loading.py")


if __name__ == "__main__":
    if in_venv():
        print(f"{red()}")
        missing = check_dependencies()
        if missing:
            print_install_instructions(missing)
            sys.exit(1)
    else:
        print(f"{blue()}")
