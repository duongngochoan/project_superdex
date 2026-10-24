/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;

using Newtonsoft.Json;
using Newtonsoft.Json.Converters;

using CADRobotExporter.Export;

namespace CADRobotExporter.Utilities
{
    /// <summary>
    /// Reads and writes user-defined exporter configuration presets as JSON files
    /// under the current user's profile.
    /// </summary>
    [ComVisible(false)]
    public static class PresetStore
    {
        /// <summary>
        /// Returns the presets directory, creating it if it does not already exist.
        /// </summary>
        public static string GetPresetsDirectory()
        {
            string path = Path.Combine(
                System.Environment.GetFolderPath(System.Environment.SpecialFolder.UserProfile),
#if NX
                "NxRobotExporter",
#elif SOLIDWORKS
                "sw2urdf",
#endif
                "presets");

            if (!Directory.Exists(path))
                Directory.CreateDirectory(path);

            return path;
        }

        public static JsonSerializerSettings GetJsonSettings()
        {
            return new JsonSerializerSettings
            {
                ObjectCreationHandling = ObjectCreationHandling.Replace,
                Converters = new List<JsonConverter> { new StringEnumConverter() },
                Formatting = Formatting.Indented
            };
        }

        public static List<ExporterConfigurationPreset> LoadUserPresets()
        {
            var presets = new List<ExporterConfigurationPreset>();
            string presetsDir = GetPresetsDirectory();
            foreach (string file in Directory.GetFiles(presetsDir, "*.json"))
            {
                try
                {
                    string json = File.ReadAllText(file);
                    var config = JsonConvert.DeserializeObject<ExporterConfiguration>(json, GetJsonSettings());
                    presets.Add(new ExporterConfigurationPreset
                    {
                        Name = Path.GetFileNameWithoutExtension(file),
                        Configuration = config,
                        IsUserPreset = true
                    });
                }
                catch
                {
                    // Skip invalid files
                }
            }
            return presets;
        }

        /// <summary>
        /// Strips characters that are illegal in a file name, then writes the configuration
        /// as JSON. Returns the sanitized preset name actually used.
        /// </summary>
        public static string SaveUserPreset(string presetName, ExporterConfiguration config)
        {
            string invalidChars = new string(Path.GetInvalidFileNameChars());
            foreach (char c in invalidChars)
                presetName = presetName.Replace(c.ToString(), "");

            string filePath = Path.Combine(GetPresetsDirectory(), presetName + ".json");
            string json = JsonConvert.SerializeObject(config, GetJsonSettings());
            File.WriteAllText(filePath, json);

            return presetName;
        }
    }
}
