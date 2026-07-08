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
"""Make the package importable without installing it.

The package root is added to ``sys.path`` so ``cube_petit_python_api`` can be
imported directly; its ``__init__`` is lazy (PEP 562), so pure-logic modules
such as ``petit_names`` work without a ROS environment.

The utils directory itself is also added because ``test_gpt_logic`` imports
``gpt_logic`` as a top-level module (it predates the lazy ``__init__``).
"""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'cube_petit_python_api' / 'utils'))
