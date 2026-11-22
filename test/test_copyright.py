# Copyright 2026 The turtlebot3-waypoint-navigation authors
#
# Licensed under the MIT License

from ament_copyright.main import main
import pytest

@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0
