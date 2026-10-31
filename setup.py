"""Build hook only; the project metadata is in pyproject.toml.

Copies the canonical markdown (RULES.md, GUIDE.md, SKILLS.md, reference/*.md, the report
template) into dflog/docs/ inside the built package, so `alog docs` works from a wheel.
The list of files is `dflog/docs.py::sources()`, loaded with runpy because the package's
dependencies are not installed at build time. Nothing is copied into the source tree.
"""
import os
import runpy
import shutil

from setuptools import setup
from setuptools.command.build_py import build_py

HERE = os.path.dirname(os.path.abspath(__file__))


class build_py_with_docs(build_py):
    def run(self):
        super().run()
        docs = runpy.run_path(os.path.join(HERE, "dflog", "docs.py"))
        out = os.path.join(self.build_lib, "dflog", "docs")
        os.makedirs(out, exist_ok=True)
        for topic, src in docs["sources"](HERE).items():
            shutil.copyfile(src, os.path.join(out, topic + ".md"))


setup(cmdclass={"build_py": build_py_with_docs})
