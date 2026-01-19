#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A shim of `std::format` for older compilers (it is available since GCC 13).
 *
 * If std::format is not available, use fmt library.
 *
 * \author Martin Pecka
 */

#if __has_include(<format>)
#include <format>
#if __cpp_lib_format >= 201907L
#define CRAS_HAS_STD_FORMAT
#endif
#endif

#ifdef CRAS_HAS_STD_FORMAT

namespace cras
{
using ::std::format;
using ::std::format_to;
using ::std::format_to_n;
using ::std::formatted_size;
using ::std::basic_format_string;
using ::std::format_string;
using ::std::vformat;
using ::std::vformat_to;
using ::std::make_format_args;
using ::std::visit_format_arg;
using ::std::formatter;
using ::std::basic_format_arg;
using ::std::basic_format_args;
using ::std::format_args;
using ::std::basic_format_context;
using ::std::format_context;
using ::std::basic_format_parse_context;
using ::std::format_parse_context;
using ::std::format_error;
}

#else

#include <fmt/chrono.h>
#include <fmt/format.h>

namespace cras
{
using ::fmt::format;
using ::fmt::format_to;
using ::fmt::format_to_n;
using ::fmt::formatted_size;
using ::fmt::basic_format_string;
using ::fmt::format_string;
using ::fmt::vformat;
using ::fmt::vformat_to;
using ::fmt::make_format_args;
using ::fmt::visit_format_arg;
using ::fmt::formatter;
using ::fmt::basic_format_arg;
using ::fmt::basic_format_args;
using ::fmt::format_args;
using ::fmt::basic_format_context;
using ::fmt::format_context;
using ::fmt::basic_format_parse_context;
using ::fmt::format_parse_context;
using ::fmt::format_error;
}

#endif
