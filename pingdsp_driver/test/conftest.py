"""pytest configuration for pingdsp_driver tests.

Mirrors serial_ping_pkg: make the sibling harness importable regardless of how
pytest is invoked (``python3 -m pytest test/`` or ``colcon test``), and reap any
subprocess the full-stack harness spawned after each test so a stray driver from
one test cannot interfere with the next.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))


@pytest.fixture(autouse=True)
def _reap_full_stack_processes():
    """Kill any harness-spawned subprocess after each test."""
    yield
    try:
        import full_stack_harness
        full_stack_harness.reap_spawned()
    except Exception:
        pass
