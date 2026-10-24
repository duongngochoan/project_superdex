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

#include <superdex_physics.h>

#include <mochi_core/utils/error.h>
#include <mochi_core/utils/span.h>
#include <mochi_core/utils/transform_rt.h>

#include <string_view>

namespace superdex::robotics {

struct ControllerScalarValues {
  std::string_view name;
  mochi::Span<mochi::real const> values;
};

/* @brief Backend-neutral destination for controller recording data. */
class MOCHI_API ControllerRecordingSink {
 public:
  ControllerRecordingSink() = default;
  virtual ~ControllerRecordingSink();

  MOCHI_DECLARE_MOVE_ONLY(ControllerRecordingSink)

  virtual void WriteConstantScalars(
      std::string_view name,
      mochi::Span<mochi::real const> values,
      mochi::Error& error) = 0;
  virtual void WriteConstantTransform(
      std::string_view name,
      mochi::TransformRT const& value,
      mochi::Error& error) = 0;
  virtual void WriteScalars(
      std::string_view seriesName,
      mochi::Span<ControllerScalarValues const> fields,
      mochi::Error& error) = 0;
  virtual void WriteTransform(
      std::string_view seriesName,
      mochi::TransformRT const& value,
      mochi::Error& error) = 0;
};

} // namespace superdex::robotics
