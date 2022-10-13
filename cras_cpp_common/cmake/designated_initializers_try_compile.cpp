/**
 * \file
 * \brief Test availability of designated struct initializers.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

struct S
{
  int a;
  int b;
};

void fn()
{
  S s{.b = 1};
}