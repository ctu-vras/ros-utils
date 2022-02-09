/**
 * \file
 * \brief Utilities for working with threads.
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#pragma once

#include <string>

namespace cras
{

/**
 * \brief Get the OS name of the current thread.
 * \return Name of the thread (up to 15 characters).
 * \note This function is actually pretty fast. It can be called more than 1 million times per second.
 */
std::string getThreadName();

/**
 * \brief Set the OS name of the current thread.
 * \param[in] name The name to set.
 * \note The name will be automatically shortened to be 15 chars max.
 * \note This function is actually pretty fast. It can be called more than 1 million times per second.
 */
void setThreadName(const std::string& name);

}