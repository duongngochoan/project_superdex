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

#include <superdex_recorder/robotics_recorder.h>

#include <superdex_recorder/series.h>

#include <algorithm>
#include <array>
#include <unordered_set>
#include <utility>

namespace superdex::recorder {
namespace {

constexpr std::string_view kControllerArchetype = "superdex.archetypes.Controller";
constexpr std::string_view kRobotCommandArchetype = "superdex.archetypes.RobotCommand";
constexpr std::string_view kAppliedCommandStreamName = "applied_command";

ComponentField ControllerField(std::string component) {
  return {std::string{kControllerArchetype}, "superdex.robotics." + std::move(component)};
}

ComponentField RobotField(std::string component) {
  return {std::string{kRobotCommandArchetype}, "superdex.robotics." + std::move(component)};
}

std::string UniqueControllerId(
    robotics::ControllerBase const& controller,
    std::unordered_set<std::string>& ids) {
  std::string base = controller.GetName().empty() ? std::string{controller.GetTypeName()}
                                                  : std::string{controller.GetName()};
  if (base.empty()) {
    base = "controller";
  }
  std::string id = base;
  for (uint32_t suffix = 1; !ids.insert(id).second; ++suffix) {
    id = base + "_" + std::to_string(suffix);
  }
  return id;
}

} // namespace

detail::ControllerSink::ControllerSink(Episode& episode, std::string controllerPath)
    : _episode{episode}, _controllerPath{std::move(controllerPath)} {}

void detail::ControllerSink::SetTime(SampleTime time) {
  _time = time;
}

void detail::ControllerSink::WriteConstantScalars(
    std::string_view name,
    mochi::Span<mochi::real const> values,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  std::vector<double> converted;
  converted.reserve(values.size());
  for (mochi::real const value : values) {
    converted.push_back(static_cast<double>(value));
  }
  _episode.WriteConstantScalars(
      _controllerPath,
      ControllerField(std::string{name}),
      mochi::Span<double const>{converted.data(), converted.size()},
      error);
}

void detail::ControllerSink::WriteConstantTransform(
    std::string_view name,
    mochi::TransformRT const& value,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  std::string const prefix{name};
  _episode.WriteConstantPose(
      _controllerPath + "/" + detail::EncodeEntityPathSegment(name),
      ControllerField(prefix + "_translation_xyz"),
      ControllerField(prefix + "_quaternion_xyzw"),
      value,
      error);
}

void detail::ControllerSink::WriteScalars(
    std::string_view seriesName,
    mochi::Span<robotics::ControllerScalarValues const> fields,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  if (!_time.has_value()) {
    MOCHI_ERROR_SET(error, "Controller sample has no timestamp.");
    return;
  }
  MOCHI_ERROR_IF(fields.empty(), error, "Controller scalar sample has no fields.");
  MOCHI_ERROR_RETURN(error);
  SampleTime const time = *_time;

  auto binding =
      std::find_if(_scalarBindings.begin(), _scalarBindings.end(), [&](ScalarBinding const& value) {
        return value.name == seriesName;
      });
  if (binding == _scalarBindings.end()) {
    MOCHI_ERROR_IF_NOT(
        std::all_of(
            fields.begin(), fields.end(), [](auto const& field) { return !field.name.empty(); }),
        error,
        "Controller scalar field name must not be empty.");
    MOCHI_ERROR_RETURN(error);

    std::vector<ComponentField> descriptors;
    std::vector<std::string> fieldNames;
    std::vector<mochi::Span<mochi::real const>> valueSpans;
    descriptors.reserve(fields.size());
    fieldNames.reserve(fields.size());
    valueSpans.reserve(fields.size());
    for (auto const& field : fields) {
      fieldNames.emplace_back(field.name);
      descriptors.push_back(ControllerField(std::string{field.name}));
    }
    auto* const series = _episode.CreateScalarSeries(
        _controllerPath + "/" + detail::EncodeEntityPathSegment(seriesName), descriptors, error);
    MOCHI_ERROR_RETURN(error);
    _scalarBindings.push_back({
        .name = std::string{seriesName},
        .fieldNames = std::move(fieldNames),
        .valueSpans = std::move(valueSpans),
        .series = series,
    });
    binding = std::prev(_scalarBindings.end());
  } else {
    bool const fieldsMatch = fields.size() == binding->fieldNames.size() &&
        std::equal(fields.begin(),
                   fields.end(),
                   binding->fieldNames.begin(),
                   [](auto const& field, std::string const& name) { return field.name == name; });
    MOCHI_ERROR_IF_NOT(fieldsMatch, error, "Controller scalar series fields changed.");
    MOCHI_ERROR_RETURN(error);
  }

  binding->valueSpans.clear();
  for (auto const& field : fields) {
    binding->valueSpans.push_back(field.values);
  }
  binding->series->Append(time, binding->valueSpans, error);
}

void detail::ControllerSink::WriteTransform(
    std::string_view seriesName,
    mochi::TransformRT const& value,
    mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  if (!_time.has_value()) {
    MOCHI_ERROR_SET(error, "Controller sample has no timestamp.");
    return;
  }
  SampleTime const time = *_time;
  auto binding = std::find_if(
      _transformBindings.begin(), _transformBindings.end(), [&](TransformBinding const& item) {
        return item.name == seriesName;
      });
  if (binding == _transformBindings.end()) {
    std::string const prefix{seriesName};
    auto* const series = _episode.CreateScalarSeries(
        _controllerPath + "/" + detail::EncodeEntityPathSegment(seriesName),
        {ControllerField(prefix + "_translation_xyz"),
         ControllerField(prefix + "_quaternion_xyzw")},
        error);
    MOCHI_ERROR_RETURN(error);
    _transformBindings.push_back({std::string{seriesName}, series});
    binding = std::prev(_transformBindings.end());
  }
  auto const translation = value.GetTranslation();
  auto const& rotation = value.GetRotation();
  std::array<mochi::real, 4> const quaternion{
      rotation.data[0], rotation.data[1], rotation.data[2], rotation.data[3]};
  std::array<mochi::Span<mochi::real const>, 2> const fields{{
      {translation.data(), 3},
      {quaternion.data(), quaternion.size()},
  }};
  binding->series->Append(time, fields, error);
}

detail::Controller::Controller(robotics::ControllerHandle handle, detail::ControllerSink&& sink)
    : handle{handle}, sink{std::move(sink)} {}

RoboticsRecorder::RoboticsRecorder(Episode& episode, robotics::Bot& robot, mochi::Error& error)
    : _context{*robot.GetBotContext()}, _robotHandle{robot.GetHandle()} {
  mochi::Actor* const actor = robot.GetArticulatedActor();
  if (actor == nullptr) {
    MOCHI_ERROR_SET(error, "Robot has no articulated actor.");
    return;
  }
  _numDofs = static_cast<size_t>(actor->GetNumDofs());
  _appliedEffort.resize(_numDofs);

  std::string const robotId = detail::RobotEntityId(actor->GetName());
  std::string const robotPath = "data/robotics/robots/" + detail::EncodeEntityPathSegment(robotId);
  _appliedEffortSeries = episode.CreateScalarSeries(
      robotPath + "/" + std::string{kAppliedCommandStreamName},
      {RobotField("applied_joint_effort")},
      error);
  MOCHI_ERROR_RETURN(error);
  episode.WriteConstantText(robotPath, RobotField("robot_id"), robotId, error);
  char const* const prefabName = robot.GetName();
  episode.WriteConstantText(
      robotPath, RobotField("robot_prefab_name"), prefabName == nullptr ? "" : prefabName, error);
  std::array<double, 1> const numDofs{static_cast<double>(_numDofs)};
  episode.WriteConstantScalars(robotPath, RobotField("num_dofs"), numDofs, error);

  std::unordered_set<std::string> ids;
  auto const handles = robot.GetControllerHandles();
  _controllers.reserve(handles.size());
  for (auto const handle : handles) {
    auto* const controller = robot.GetController(handle, error);
    MOCHI_ERROR_RETURN(error);
    if (controller == nullptr) {
      MOCHI_ERROR_SET(error, "Robot has an invalid controller handle.");
      return;
    }

    std::string const id = UniqueControllerId(*controller, ids);
    std::string const path = robotPath + "/controllers/" + detail::EncodeEntityPathSegment(id);
    episode.WriteConstantText(path, ControllerField("controller_id"), id, error);
    episode.WriteConstantText(
        path, ControllerField("controller_type"), controller->GetTypeName(), error);
    detail::ControllerSink sink{episode, path};
    controller->ConfigureRecording(sink, error);
    MOCHI_ERROR_RETURN(error);
    _controllers.emplace_back(handle, std::move(sink));
  }
}

RoboticsRecorder::~RoboticsRecorder() = default;

void RoboticsRecorder::Record(SampleTime time, mochi::Error& error) {
  MOCHI_ERROR_RETURN(error);
  for (auto& recorded : _controllers) {
    robotics::ControllerBase* const controller = _context.GetController(recorded.handle);
    if (controller == nullptr) {
      MOCHI_ERROR_SET(error, "A recorded controller was destroyed.");
      return;
    }
    recorded.sink.SetTime(time);
    controller->Record(recorded.sink, error);
    MOCHI_ERROR_RETURN(error);
  }

  if (_appliedEffortSeries == nullptr) {
    MOCHI_ERROR_SET(error, "Applied effort series is unavailable.");
    return;
  }
  robotics::Bot* const robot = _context.GetBot(_robotHandle);
  mochi::Actor* const actor = robot != nullptr ? robot->GetArticulatedActor() : nullptr;
  if (actor == nullptr) {
    MOCHI_ERROR_SET(error, "The recorded robot was destroyed.");
    return;
  }
  actor->GetExternalForces(_appliedEffort, error);
  MOCHI_ERROR_RETURN(error);
  std::array<mochi::Span<mochi::real const>, 1> const fields{
      mochi::Span<mochi::real const>{_appliedEffort.data(), _appliedEffort.size()}};
  _appliedEffortSeries->Append(time, fields, error);
}

} // namespace superdex::recorder
