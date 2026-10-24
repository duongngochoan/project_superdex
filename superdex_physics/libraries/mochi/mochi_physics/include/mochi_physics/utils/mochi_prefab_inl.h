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

#include <mochi_physics/utils/mochi_prefab.h>

namespace mochi::prefab {

// Convenience overloads of @ref AddToScene that delegate to the canonical overloads using default
// @ref PrefabParams. Declared in mochi_prefab.h.
inline AddToSceneResult AddToScene(ScenePrefab const& prefab, Scene* scene, Error& error) {
  return AddToScene(prefab, scene, PrefabParams{}, error);
}

inline AddToSceneResult
AddToScene(std::string_view prefabPath, std::string_view rootPath, Scene* scene, Error& error) {
  return AddToScene(prefabPath, rootPath, scene, PrefabParams{}, error);
}

// Convenience overloads of @ref ExportScene and @ref ExportActor that use default
// @ref ExportParams. Declared in mochi_prefab.h.
inline void ExportScene(
    Scene const* scene,
    std::string_view exportName,
    std::string_view outputDir,
    Error& error) {
  ExportScene(scene, exportName, outputDir, ExportParams{}, error);
}

inline void ExportActor(
    Actor const* actor,
    std::string_view exportName,
    std::string_view outputDir,
    Error& error) {
  ExportActor(actor, exportName, outputDir, ExportParams{}, error);
}

} // namespace mochi::prefab
