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

#include <superdex_recorder/export.h>

#include <mochi_core/utils/error.h>
#include <mochi_physics/cpp_api/mochi_scene.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace superdex::recorder {

/* @brief Contact points of consecutive frames in the recorded contact layout.
 * Each contact appears once, body A is the lower body index, the force acts on body A, and
 * contacts are sorted by body pair. Positions and forces are world-space xyz triples. */
struct ContactColumns {
  std::vector<uint32_t> frameLengths;
  std::vector<mochi::real> bodyA;
  std::vector<mochi::real> bodyB;
  std::vector<mochi::real> positionA;
  std::vector<mochi::real> positionB;
  std::vector<mochi::real> forceOnA;

  void Clear();
};

struct ContactBody {
  std::string path;
  mochi::ActorHandle handle;
};

/* @brief Reads the contact points of a fixed set of bodies after each step.
 * Bodies are indexed in path order. The scene must outlive the collector. */
class SUPERDEX_RECORDER_API ContactCollector final {
  MOCHI_DECLARE_NO_COPY_NO_MOVE(ContactCollector)

 public:
  ContactCollector(mochi::Scene& scene, std::vector<ContactBody> bodies, mochi::Error& error);
  ~ContactCollector();

  /* Appends the contacts of the latest step as one frame. Fails if a body was destroyed. */
  void Collect(ContactColumns& output, mochi::Error& error) const;

  [[nodiscard]] std::vector<ContactBody> const& GetBodies() const;

 private:
  size_t FindBody(mochi::ActorHandle handle, mochi::Error& error) const;

  mochi::Scene& _scene;
  std::vector<ContactBody> _bodies;
  std::vector<mochi::QueryHandle> _queries;
  std::map<mochi::ActorHandle, size_t> _indices;
};

} // namespace superdex::recorder
