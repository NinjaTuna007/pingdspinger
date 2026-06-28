"""Root pytest config for pingdsp_driver.

Keep collection out of ``launch/``: ``test_driver.launch.py`` matches pytest's
``test_*`` pattern and trips the launch_testing import hook, which would abort
collection of the real unit/integration tests under ``test/``.
"""

collect_ignore_glob = ['launch/*', 'scripts/*']
