<!-- SPDX-License-Identifier: BSD-3-Clause -->
<!-- SPDX-FileCopyrightText: Czech Technical University in Prague -->

# cras_lint

Linters for Python and C++ preconfigured to follow the CRAS coding style (mostly line length 120 + SPDX licenses).

## Usage

Just `<test_depend>cras_lint</test_depend>` in your `package.xml`, and then add these lines to `CMakeLists.txt`:

```CMake
if(BUILD_TESTING)
  find_package(cras_lint REQUIRED)
  cras_lint_common()
  cras_lint_cpp()  # If you have C++ files
  cras_lint_py()  # If you have Python files
endif()
```
