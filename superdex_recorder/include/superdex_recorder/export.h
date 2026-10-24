/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <mochi_core/mochi_platform.h>

/* Callers of the shared library may use a different heap, such as Unreal's FMemory. Exported APIs
 * must not pass heap ownership across the boundary: take heap-owning types by const reference, and
 * define constructors and destructors out of line. */
#if defined(SUPERDEX_RECORDER_SHARED) && SUPERDEX_RECORDER_SHARED
#if MOCHI_PLATFORM_WINDOWS
#if defined(SUPERDEX_RECORDER_EXPORTS) && SUPERDEX_RECORDER_EXPORTS
#define SUPERDEX_RECORDER_API __declspec(dllexport)
#else
#define SUPERDEX_RECORDER_API __declspec(dllimport)
#endif
#else
#define SUPERDEX_RECORDER_API __attribute__((visibility("default")))
#endif
#else
#define SUPERDEX_RECORDER_API
#endif
