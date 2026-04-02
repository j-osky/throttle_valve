#ifndef SLSA_ENGINE_H
#define SLSA_ENGINE_H

/**
 * @file slsa_engine.h
 *
 * This file contains macros to control the scoping of the exported classes
 * and functions from this module. This file should be included by all the
 * in this module.
 *
 */

/* Copyright 2011-2023 The MathWorks, Inc. */

#ifdef BUILDING_LIBMWSLSA_ENGINE
/* being included from inside SLSA_ENGINE module */
#include "package.h"
#define SLSA_ENGINE_PUBLISHED_C extern "C" DLL_EXPORT_SYM
#else
/* being included from outside, such as raccel code */
#if defined(__cplusplus)
/* Needed for non-inlined C++ S-functions */
#define SLSA_ENGINE_PUBLISHED_C extern "C"
#else
#define SLSA_ENGINE_PUBLISHED_C extern
#endif
#endif

#endif // SLSA_ENGINE_H
