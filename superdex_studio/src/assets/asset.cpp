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

#include "assets/asset.h"
#include "assets/asset_manager.h"
#include "editors/asset_editor.h"

#include <mochi_renderer/resource_manager.h>

#include <mochi_core/utils/log.h>

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace superdex::studio {

//--------------------------------------------------------------------------------------------------
// ASSET
//--------------------------------------------------------------------------------------------------

std::string const& Asset::GetName() const {
  return _name;
}

mochi::Path const& Asset::GetPath() const {
  return _path;
}

AssetType Asset::GetType() const {
  return _type;
}

ImU32 Asset::GetColor() const {
  return GetAssetTypeColor(_type);
}

char const* Asset::GetTypeLabel() const {
  return GetAssetTypeLabel(_type);
}

bool Asset::RendersThumbnail() const {
  return false;
}

void Asset::StageThumbnailScene(mochi_renderer::Scene& /*scene*/) {}

void Asset::MarkThumbnailDirty() {
  _thumbDirty = true;
}

void* Asset::GetThumbnailImage() const {
  if (RendersThumbnail()) {
    return _thumbRendered ? _thumbRenderTarget->GetTextureId() : nullptr;
  }
  return nullptr;
}

void Asset::SetDirty(bool dirty) {
  _dirty = dirty;
}

bool Asset::IsDirty() const {
  return _dirty;
}

bool Asset::IsReadOnly() const {
  return _readOnly;
}

void Asset::SetReadOnly(bool readOnly) {
  _readOnly = readOnly;
}

bool Asset::IsSavable() const {
  return false;
}

bool Asset::Save() {
  return false;
}

bool Asset::SupportsSaveAs() const {
  return false;
}

bool Asset::SaveAs(mochi::Path const& /*path*/) {
  return false;
}

bool Asset::ReloadFromDisk() {
  return false;
}

int Asset::GetReferenceCount() const {
  return _manager->GetPathReferenceCount(_path);
}

std::vector<IAssetReferencer*> Asset::GetReferencers() const {
  return _manager->GetReferencersToPath(_path);
}

std::unique_ptr<AssetEditor> Asset::CreateEditor(SuperDexStudio*) {
  return nullptr;
}

Asset::Asset(
    std::string const& name,
    mochi::Path const& path,
    AssetType type,
    AssetManager* manager)
    : _manager(manager), _name(name), _path(path), _type(type) {}

void Asset::OnUnload(mochi_renderer::ResourceManager& /*resourceManager*/) {}

void Asset::OnRewritePath(
    mochi::Path const& /*oldPath*/,
    mochi::Path const& /*newPath*/,
    mochi_renderer::ResourceManager& /*resourceManager*/) {}

void Asset::Refresh() {
  MarkThumbnailDirty();
}

} // namespace superdex::studio
