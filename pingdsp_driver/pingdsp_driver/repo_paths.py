#!/usr/bin/env python3
"""Portable data-directory resolution for the pingdsp driver nodes.

Keeps generated output (point clouds, saved images, bags, ...) inside the
repo's gitignored folders by default instead of the user's home directory,
while still allowing an explicit override via an environment variable or a node
parameter. This is what lets a fresh clone on any machine/user "just work".
"""

import os


def repo_root():
    """Best-effort absolute path to the pingdspinger repo root, or ``None``.

    Resolution order:
      1. ``$PINGDSP_REPO`` if it points at a real directory.
      2. Walk up from this file (works for a source checkout or a
         ``--symlink-install`` overlay, where ``__file__`` resolves back into
         the repo) until a directory named ``pingdspinger`` that also contains
         ``pingdsp_driver`` is found.
    """
    env = os.environ.get('PINGDSP_REPO')
    if env and os.path.isdir(env):
        return os.path.realpath(env)
    d = os.path.dirname(os.path.realpath(__file__))
    for _ in range(6):
        d = os.path.dirname(d)
        if os.path.basename(d) == 'pingdspinger' \
                and os.path.isdir(os.path.join(d, 'pingdsp_driver')):
            return d
    return None


def data_dir(subdir='', create=True):
    """Resolve an output directory, preferring the repo over ``$HOME``.

    The base is ``$PINGDSP_DATA_DIR`` if set, else the repo root, else
    ``~/sonar_data`` as a last resort (e.g. a plain non-symlink install run
    without either the env var or a discoverable repo). The path is expanded and
    made absolute, and created (``exist_ok=True``) unless ``create=False`` - so
    a single call is the one source of truth for both resolving *and* ensuring
    the directory, regardless of whether it lands in the repo or an external
    drive. Creation failures are swallowed (the caller/tool will surface them).
    """
    base = os.environ.get('PINGDSP_DATA_DIR') or repo_root() \
        or os.path.expanduser('~/sonar_data')
    base = os.path.expanduser(base)
    path = os.path.realpath(os.path.join(base, subdir) if subdir else base)
    if create:
        try:
            os.makedirs(path, exist_ok=True)
        except OSError:
            pass
    return path
