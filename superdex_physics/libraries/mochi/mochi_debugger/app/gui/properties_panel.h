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

#include <mochi_physics/mochi_physics_experimental.h>

namespace mochi::dbg {

struct UiState;

struct IslandsPropertiesState {
  // Sliders change the value every frame, but it is only sent to the server on release. While a
  // widget is active, the UI shows this buffer instead of the client's value.
  experimental::SleepParams edit;
  bool editing = false;
};

// Properties panel: rendering settings (origin, meshes, lights, materials) and island sleeping.
void BuildPropertiesPanel(UiState& state);

} // namespace mochi::dbg
