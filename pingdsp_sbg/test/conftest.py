"""pytest configuration for pingdsp_sbg tests.

Make the sibling harness importable and reap any spawned node after each test.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))


@pytest.fixture(autouse=True)
def _reap_spawned_processes():
    """Kill any harness-spawned subprocess after each test."""
    yield
    try:
        import sbg_harness
        sbg_harness.reap_spawned()
    except Exception:
        pass
