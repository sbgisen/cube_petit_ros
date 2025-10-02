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

import pathlib
import time
import xml.etree.ElementTree as ET

file_path = pathlib.Path(__file__).absolute()
tree = ET.parse(file_path.parent.parent / 'package.xml')
root = tree.getroot()
package_name = root.find('name').text

install_path = file_path.parent.parent
for parent in file_path.parents:
    if (parent / 'install').is_dir():
        install_path = parent / 'install'
        break
pkg_install_path = install_path / package_name
project_file = pkg_install_path / 'share' / package_name / 'pyproject.toml'
scripts_path = pkg_install_path / 'lib' / package_name

while not project_file.is_file():
    time.sleep(0.1)
if project_file.is_symlink():
    project_file = project_file.resolve()
shebang = f'#!/usr/bin/env -S uv run --script --project {project_file}'
for file in scripts_path.iterdir():
    if not file.is_file():
        continue
    with open(file, encoding='utf-8') as f:
        lines = f.readlines()
    lines[0] = shebang + '\n'
    with open(file, 'w', encoding='utf-8') as f:
        f.writelines(lines)