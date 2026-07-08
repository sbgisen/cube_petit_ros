#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2026 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Sanity tests for config and launch files (no ROS environment required)."""

from pathlib import Path
import py_compile

import pytest
import yaml

PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_YAML_FILES = sorted((PACKAGE_DIR / 'config').glob('*.yaml'))
LAUNCH_PY_FILES = sorted((PACKAGE_DIR / 'launch').glob('*.launch.py'))


def test_config_yaml_files_exist() -> None:
    """The package should ship at least one config/*.yaml file."""
    assert CONFIG_YAML_FILES, f'No *.yaml files found under {PACKAGE_DIR / "config"}'


def test_launch_py_files_exist() -> None:
    """The package should ship at least one launch/*.launch.py file."""
    assert LAUNCH_PY_FILES, f'No *.launch.py files found under {PACKAGE_DIR / "launch"}'


@pytest.mark.parametrize('yaml_path', CONFIG_YAML_FILES, ids=lambda p: p.name)
def test_config_yaml_parses_and_is_non_empty(yaml_path: Path) -> None:
    """Every config/*.yaml must parse as YAML and contain data."""
    data = yaml.safe_load(yaml_path.read_text(encoding='utf-8'))
    assert data is not None, f'{yaml_path.name} is empty'
    assert data, f'{yaml_path.name} parsed to an empty value'


@pytest.mark.parametrize('launch_path', LAUNCH_PY_FILES, ids=lambda p: p.name)
def test_launch_py_compiles(launch_path: Path) -> None:
    """Every launch/*.launch.py must be valid Python (byte-compilable)."""
    py_compile.compile(str(launch_path), doraise=True)
