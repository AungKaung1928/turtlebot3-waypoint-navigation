# Copyright 2025 Aung Kaung Myat
#
# Licensed under the MIT License

from ament_copyright.main import main
import pytest

@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0
