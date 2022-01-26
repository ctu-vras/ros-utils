#pragma once

/**
 * \file
 * \brief Filter utils aggregate header. It is deprecated.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#ifdef _MSC_VER
#pragma message("Using deprecated header filter_utils.hpp. Use filter_base.hpp or filter_chain.hpp instead.")
#else
#warning Using deprecated header filter_utils.hpp. Use filter_base.hpp or filter_chain.hpp instead.
#endif

#include "filter_base.hpp"
#include "filter_chain.hpp"