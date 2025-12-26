#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
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
#

from __future__ import annotations

from pathlib import Path
from typing import Dict

import yaml


class PlacesStore:
    """Handles persistent storage of places.yaml."""

    def __init__(self, yaml_path: Path) -> None:
        self._path = yaml_path
        self._data: Dict = {}

        self._load()

    def _load(self) -> None:
        if self._path.exists():
            with self._path.open() as f:
                self._data = yaml.safe_load(f) or {}
        else:
            self._data = {}

    def save_place(
        self,
        category: str,
        name: str,
        pose: list[float],
        room: str | None,
    ) -> None:
        section = self._data.setdefault(category, {})
        section.setdefault('order', [])
        places = section.setdefault('places', {})

        places[name] = {
            'pose': pose,
            'room': room,
        }

        if name not in section['order']:
            section['order'].append(name)

        with self._path.open('w') as f:
            yaml.safe_dump(self._data, f, sort_keys=False)
