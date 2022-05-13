#pragma once

/**
 * \file
 * \brief Diagnostic updater that automatically sets its Hardware ID to hostname of the machine (deprecated backwards
 * compatibility header, use diag_utils/updater.h instead).
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include "../diag_utils/updater.h"

namespace diagnostic_updater
{
//! \brief Backwards compatibility typedef.
[[deprecated("Use cras::DiagnosticUpdater instead")]]
typedef ::cras::DiagnosticUpdater ImprovedUpdater;
}

