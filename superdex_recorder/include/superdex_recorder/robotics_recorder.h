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

#include <superdex_recorder/recorder.h>

#include <superdex_robotics/controllers/controller_base.h>
#include <superdex_robotics/controllers/controller_recording.h>
#include <superdex_robotics/superdex_robotics.h>

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace superdex::recorder {

namespace detail {

struct ScalarBinding {
  std::string name;
  std::vector<std::string> fieldNames;
  /* Reused to avoid allocating the per-field views on every sample. */
  std::vector<mochi::Span<mochi::real const>> valueSpans;
  ScalarSeries* series = nullptr;
};

struct TransformBinding {
  std::string name;
  ScalarSeries* series = nullptr;
};

class ControllerSink final : public robotics::ControllerRecordingSink {
 public:
  ControllerSink(Episode& episode, std::string controllerPath);

  void SetTime(SampleTime time);

  void WriteConstantScalars(
      std::string_view name,
      mochi::Span<mochi::real const> values,
      mochi::Error& error) override;
  void WriteConstantTransform(
      std::string_view name,
      mochi::TransformRT const& value,
      mochi::Error& error) override;
  void WriteScalars(
      std::string_view seriesName,
      mochi::Span<robotics::ControllerScalarValues const> fields,
      mochi::Error& error) override;
  void WriteTransform(
      std::string_view seriesName,
      mochi::TransformRT const& value,
      mochi::Error& error) override;

 private:
  Episode& _episode;
  std::string _controllerPath;
  std::optional<SampleTime> _time;
  std::vector<ScalarBinding> _scalarBindings;
  std::vector<TransformBinding> _transformBindings;
};

struct Controller {
  Controller(robotics::ControllerHandle handle, ControllerSink&& sink);

  robotics::ControllerHandle handle;
  ControllerSink sink;
};

} // namespace detail

/* Records the controllers owned by one SuperDex robot, and the joint effort applied to it.
 * The episode and the robot's RoboticsContext must outlive this recorder. Construction fails if
 * the episode already records a robot with the same articulated actor name. Record fails if the
 * robot or a recorded controller is destroyed. */
class SUPERDEX_RECORDER_API RoboticsRecorder final {
  MOCHI_DECLARE_NO_COPY_NO_MOVE(RoboticsRecorder)

 public:
  RoboticsRecorder(Episode& episode, robotics::Bot& robot, mochi::Error& error);
  ~RoboticsRecorder();

  /* Writes each controller's latest target and output, and the external forces Mochi applies to
   * the robot as its applied joint effort. Mochi keeps external forces until they are replaced, so
   * call this after applying the step's effort. */
  void Record(SampleTime time, mochi::Error& error);

 private:
  robotics::RoboticsContext& _context;
  robotics::BotHandle _robotHandle;
  size_t _numDofs = 0;
  std::vector<detail::Controller> _controllers;
  ScalarSeries* _appliedEffortSeries = nullptr;
  std::vector<mochi::real> _appliedEffort;
};

} // namespace superdex::recorder
