from ament_copyright.main import main
import pytest


@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[
        '.',
        '--exclude',
        'tests/test_copyright.py',
        'tests/test_flake8.py',
    ])
    assert rc == 0, 'Found errors'
