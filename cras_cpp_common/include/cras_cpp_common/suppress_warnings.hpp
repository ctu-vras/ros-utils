#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Preprocessor macros to supress compiler warnings for a part of a file.
 * \author Martin Pecka
 */

/**
 * \def CRAS_IGNORE_DEPRECATED_WARNING_BEGIN
 * \brief Ignore warnings about using deprecated declarations in the code following this macro. Do not forget to call
 *        `CRAS_IGNORE_DEPRECATED_WARNING_END` when the warnings should be restored.
 * \sa CRAS_IGNORE_DEPRECATED_WARNING_END
 */

/**
 * \def CRAS_IGNORE_DEPRECATED_WARNING_END
 * \brief Resume reporting warnings about using deprecated declarations in the code following this macro. This has to be
 *        called after `CRAS_IGNORE_DEPRECATED_WARNING_START` when ignoring the warnings is no longer needed.
 * \sa CRAS_IGNORE_DEPRECATED_WARNING_START
 */

#if defined __clang__

#define CRAS_IGNORE_DEPRECATED_WARNING_BEGIN \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Wdeprecated-declarations\"")
#define CRAS_IGNORE_DEPRECATED_WARNING_END \
  _Pragma("clang diagnostic pop")

#elif defined __GNUC__

#define CRAS_IGNORE_DEPRECATED_WARNING_BEGIN \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
#define CRAS_IGNORE_DEPRECATED_WARNING_END \
  _Pragma("GCC diagnostic pop")

#elif defined _MSC_VER

#define CRAS_IGNORE_DEPRECATED_WARNING_BEGIN \
  __pragma(warning(push)) \
  __pragma(warning(disable: 4996))
#define CRAS_IGNORE_DEPRECATED_WARNING_END \
  __pragma(warning(pop))

#else

#define CRAS_IGNORE_DEPRECATED_WARNING_BEGIN
#define CRAS_IGNORE_DEPRECATED_WARNING_END

#endif

/**
 * \def CRAS_IGNORE_PRINTF_SECURITY_WARNING_BEGIN
 * \brief Ignore warnings about passing variables as printf format in the code following this macro. Do not forget to
 *        call `CRAS_IGNORE_PRINTF_SECURITY_WARNING_END` when the warnings should be restored.
 * \sa CRAS_IGNORE_PRINTF_SECURITY_WARNING_END
 */

/**
 * \def CRAS_IGNORE_PRINTF_SECURITY_WARNING_END
 * \brief Resume reporting warnings about using variables as printf format in the code following this macro. This has to
 *        be called after `CRAS_IGNORE_PRINTF_SECURITY_WARNING_START` when ignoring the warnings is no longer needed.
 * \sa CRAS_IGNORE_PRINTF_SECURITY_WARNING_START
 */

#if defined __clang__

#define CRAS_IGNORE_PRINTF_SECURITY_WARNING_BEGIN \
  _Pragma("clang diagnostic push") \
  _Pragma("clang diagnostic ignored \"-Wformat-security\"")
#define CRAS_IGNORE_PRINTF_SECURITY_WARNING_END \
  _Pragma("clang diagnostic pop")

#elif defined __GNUC__

#define CRAS_IGNORE_PRINTF_SECURITY_WARNING_BEGIN \
  _Pragma("GCC diagnostic push") \
  _Pragma("GCC diagnostic ignored \"-Wformat-security\"")
#define CRAS_IGNORE_PRINTF_SECURITY_WARNING_END \
  _Pragma("GCC diagnostic pop")

#else

#define CRAS_IGNORE_PRINTF_SECURITY_WARNING_BEGIN
#define CRAS_IGNORE_PRINTF_SECURITY_WARNING_END

#endif
