#pragma once

#include <stddef.h>

/**
 * This operator allows you to write size_t literals like 5_sz.
 */
size_t operator "" _sz(unsigned long long int x)
{
  return x;
}
