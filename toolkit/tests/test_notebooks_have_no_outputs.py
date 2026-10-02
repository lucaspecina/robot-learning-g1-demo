"""Notebooks reach the repository without outputs.

The repository is public and the outputs of these notebooks include camera
images of the real setup. Outputs stay in each person's working copy; the
nbstripout Git filter removes them when committing.
"""

import json
import subprocess

import pytest

from toolkit.paths import REPO_ROOT

NOTEBOOKS = sorted(p for p in (REPO_ROOT / "notebooks").rglob("*.ipynb") if ".ipynb_checkpoints" not in p.parts)


def _git(*args: str) -> str:
    return subprocess.run(["git", "-C", str(REPO_ROOT), *args], capture_output=True, text=True).stdout


def test_notebooks_go_through_nbstripout():
    attributes = (REPO_ROOT / ".gitattributes").read_text()
    assert "*.ipynb filter=nbstripout" in attributes
    assert _git("config", "--get", "filter.nbstripout.clean").strip(), (
        "Falta configurar el filtro en este clon: correr scripts/setup_learning.sh"
    )


@pytest.mark.parametrize("path", NOTEBOOKS, ids=lambda p: str(p.relative_to(REPO_ROOT / "notebooks")))
def test_committed_notebook_has_no_outputs(path):
    committed = _git("show", f"HEAD:{path.relative_to(REPO_ROOT)}")
    if not committed:
        pytest.skip("todavía no commiteado")
    cells = json.loads(committed)["cells"]
    assert not [i for i, c in enumerate(cells) if c.get("outputs")]
