# Vendored Python dependencies

This directory contains source copies of the Python-only dependencies required
by `rtamt4ros2`. They are installed into the ROS package prefix so binary ROS
packages do not need to download dependencies from PyPI at build or run time.

| Directory | Upstream version | Source archive SHA-256 |
|---|---:|---|
| `rtamt/` | 0.3.5 | `ddc364ddff615dac8283cdde3e383e52e03c1e17d0e6719fa07f3929f59284a2` |
| `antlr4/` | 4.7 | `df12103a041553807e510f315542d36f48e43bdb9c444c5195ae4247cde799c7` |

Source archives:

- `https://files.pythonhosted.org/packages/source/r/rtamt/rtamt-0.3.5.tar.gz`
- `https://files.pythonhosted.org/packages/source/a/antlr4-python3-runtime/antlr4-python3-runtime-4.7.tar.gz`

Their license texts are retained in this directory. The ANTLR runtime has one
compatibility-only patch in `Lexer.py` and `Parser.py`: `TextIO` is imported
from `typing` instead of the removed `typing.io` compatibility namespace. This
keeps the 4.7 runtime usable on current Python versions without changing parser
behavior.
