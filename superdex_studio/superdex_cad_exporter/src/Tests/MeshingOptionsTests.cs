/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System.IO;
using System.Reflection;
using System.Runtime.Serialization;
using System.Text;

using CADRobotExporter.Export;
using CADRobotExporter.RobotDescription;
using Meshing;
using Newtonsoft.Json;
using Xunit;

namespace CADRobotExporter.Tests
{
    /// <summary>
    /// Covers the copying that keeps the global and per-link meshing tiers independent. The
    /// exporter configuration and every link hold their own <see cref="MeshingOptions"/>, so a
    /// missed field in <see cref="MeshingOptions.Clone"/> would silently alias the two tiers
    /// together rather than fail outright.
    /// </summary>
    public class MeshingOptionsTests
    {
        private static FieldInfo[] Fields()
        {
            return typeof(MeshingOptions).GetFields(BindingFlags.Public | BindingFlags.Instance);
        }

        /// <summary>
        /// Every field set to something no constructor or <c>[OnDeserializing]</c> default would
        /// produce, so a field left out of <see cref="MeshingOptions.Clone"/> shows up as a
        /// mismatch instead of matching by coincidence.
        /// </summary>
        private static MeshingOptions Populated()
        {
            return new MeshingOptions
            {
                linearDeflection = 1.25,
                angularDeflection = 2.5,
                scale = 3.75,
                backend = MeshingBackend.Delabella,
                edgeSampling = EdgeSampling.Uniform,
                targetEdgeLength = 4.5,
                targetEdgeLengthFraction = 0.125,
            };
        }

        /// <summary>
        /// Guards the fixture rather than the production code: adding a field to
        /// <see cref="MeshingOptions"/> without extending <see cref="Populated"/> would leave that
        /// field at its default in both the source and the clone, quietly making
        /// <see cref="Clone_CopiesEveryField"/> pass whether or not Clone copies it.
        /// </summary>
        [Fact]
        public void EveryFieldDiffersFromItsDefault_SoTheCloneTestCannotPassByCoincidence()
        {
            MeshingOptions defaults = new MeshingOptions();
            MeshingOptions populated = Populated();

            foreach (FieldInfo field in Fields())
            {
                Assert.NotEqual(field.GetValue(defaults), field.GetValue(populated));
            }
        }

        [Fact]
        public void Clone_CopiesEveryField()
        {
            MeshingOptions source = Populated();

            MeshingOptions clone = source.Clone();

            foreach (FieldInfo field in Fields())
            {
                Assert.Equal(field.GetValue(source), field.GetValue(clone));
            }
        }

        [Fact]
        public void Clone_ProducesAnIndependentInstance()
        {
            MeshingOptions source = Populated();

            MeshingOptions clone = source.Clone();
            clone.linearDeflection = 99.0;
            clone.backend = MeshingBackend.Isotropic;

            Assert.NotSame(source, clone);
            Assert.Equal(1.25, source.linearDeflection);
            Assert.Equal(MeshingBackend.Delabella, source.backend);
        }

        [Fact]
        public void ExporterMeshingOptionsClone_CopiesTheScalarSettings()
        {
            ExporterMeshingOptions source = new ExporterMeshingOptions
            {
                meshTagExtension = "glb",
                collisionMeshTagExtension = "stl",
                exportCollision = true,
            };

            ExporterMeshingOptions clone = source.Clone();

            Assert.Equal("glb", clone.meshTagExtension);
            Assert.Equal("stl", clone.collisionMeshTagExtension);
            Assert.True(clone.exportCollision);
        }

        /// <summary>
        /// Deliberately shallow: resolving a link's meshing swaps these two references on the copy,
        /// so copying them here would just be thrown away.
        /// </summary>
        [Fact]
        public void ExporterMeshingOptionsClone_KeepsTheSameMeshingOptionsInstances()
        {
            ExporterMeshingOptions source = new ExporterMeshingOptions();

            ExporterMeshingOptions clone = source.Clone();

            Assert.NotSame(source, clone);
            Assert.Same(source.visualMeshingOptions, clone.visualMeshingOptions);
            Assert.Same(source.collisionMeshingOptions, clone.collisionMeshingOptions);
        }

        private static ExporterMeshingOptions Globals()
        {
            return new ExporterMeshingOptions
            {
                meshTagExtension = "glb",
                collisionMeshTagExtension = "stl",
                exportCollision = true,
                visualMeshingOptions = new MeshingOptions { linearDeflection = 0.1 },
                collisionMeshingOptions = new MeshingOptions { linearDeflection = 0.5 },
            };
        }

        private static Link OverriddenLink(double visual, double collision)
        {
            Link link = new Link { meshingOverride = true };
            link.visualMeshingOptions.linearDeflection = visual;
            link.collisionMeshingOptions.linearDeflection = collision;
            return link;
        }

        [Fact]
        public void ResolveFor_PlainLink_UsesTheRobotWideOptions()
        {
            ExporterMeshingOptions globals = Globals();
            Link link = new Link();
            link.visualMeshingOptions.linearDeflection = 42.0;

            ExporterMeshingOptions resolved = globals.ResolveFor(link);

            Assert.Equal(0.1, resolved.visualMeshingOptions.linearDeflection);
            Assert.Equal(0.5, resolved.collisionMeshingOptions.linearDeflection);
        }

        [Fact]
        public void ResolveFor_OverriddenLink_UsesTheLinksOwnOptions()
        {
            ExporterMeshingOptions globals = Globals();

            ExporterMeshingOptions resolved = globals.ResolveFor(OverriddenLink(1.5, 2.5));

            Assert.Equal(1.5, resolved.visualMeshingOptions.linearDeflection);
            Assert.Equal(2.5, resolved.collisionMeshingOptions.linearDeflection);
        }

        [Fact]
        public void ResolveFor_CarriesTheScalarSettingsThrough()
        {
            ExporterMeshingOptions globals = Globals();

            ExporterMeshingOptions resolved = globals.ResolveFor(OverriddenLink(1.5, 2.5));

            Assert.Equal("glb", resolved.meshTagExtension);
            Assert.Equal("stl", resolved.collisionMeshTagExtension);
            Assert.True(resolved.exportCollision);
        }

        /// <summary>
        /// The exporter holds one bundle for the whole robot and resolves it inside a per-link loop.
        /// The original code assigned the link's options onto that shared bundle, so the first
        /// override became every later link's settings.
        /// </summary>
        [Fact]
        public void ResolveFor_DoesNotLeakOneLinksOverrideOntoTheNext()
        {
            ExporterMeshingOptions globals = Globals();

            globals.ResolveFor(OverriddenLink(1.5, 2.5));
            ExporterMeshingOptions plain = globals.ResolveFor(new Link());

            Assert.Equal(0.1, plain.visualMeshingOptions.linearDeflection);
            Assert.Equal(0.5, plain.collisionMeshingOptions.linearDeflection);
        }

        [Fact]
        public void ResolveFor_LeavesTheRobotWideOptionsUntouched()
        {
            ExporterMeshingOptions globals = Globals();
            MeshingOptions visualBefore = globals.visualMeshingOptions;
            MeshingOptions collisionBefore = globals.collisionMeshingOptions;

            globals.ResolveFor(OverriddenLink(1.5, 2.5));

            Assert.Same(visualBefore, globals.visualMeshingOptions);
            Assert.Same(collisionBefore, globals.collisionMeshingOptions);
            Assert.Equal(0.1, globals.visualMeshingOptions.linearDeflection);
            Assert.Equal(0.5, globals.collisionMeshingOptions.linearDeflection);
        }

        [Fact]
        public void ResolveFor_ANullLink_UsesTheRobotWideOptions()
        {
            ExporterMeshingOptions globals = Globals();

            Assert.Same(globals, globals.ResolveFor(null));
        }

        /// <summary>
        /// The old global flag was dropped rather than migrated, so a configuration saved with it on
        /// must still load, and must leave every link on the robot-wide options.
        /// </summary>
        [Fact]
        public void ExporterConfiguration_StillLoadsWithTheRetiredPerLinkMeshingFlag()
        {
            string json = @"{
                ""visualMeshingOptions"": { ""linearDeflection"": 0.2 },
                ""collisionMeshingOptions"": { ""linearDeflection"": 0.6 },
                ""perLinkMeshing"": true,
                ""meshTagExtension"": ""glb""
            }";

            ExporterConfiguration config = JsonConvert.DeserializeObject<ExporterConfiguration>(
                json, Utilities.PresetStore.GetJsonSettings());

            Assert.Equal(0.2, config.visualMeshingOptions.linearDeflection);
            Assert.Equal(0.6, config.collisionMeshingOptions.linearDeflection);
            Assert.Equal("glb", config.meshTagExtension);
        }

        /// <summary>A link deserialized before the flag existed reads as not overridden.</summary>
        [Fact]
        public void Link_DefaultsToNoMeshingOverride()
        {
            Assert.False(new Link().meshingOverride);
        }

        [Fact]
        public void MeshingOptions_SerializesInTheLegacySW2URDFNamespace()
        {
            var ser = new DataContractSerializer(typeof(MeshingOptions));
            using (var stream = new MemoryStream())
            {
                ser.WriteObject(stream, new MeshingOptions { angularDeflection = 0.5 });
                string xml = Encoding.UTF8.GetString(stream.ToArray());

                Assert.Contains("http://schemas.datacontract.org/2004/07/SW2URDF.Export", xml);
                Assert.DoesNotContain("CADRobotExporter", xml);
            }
        }

        [Fact]
        public void MeshingOptions_ReadsXmlWrittenBySW2URDF()
        {
            string xml =
                "<MeshingOptions xmlns=\"http://schemas.datacontract.org/2004/07/SW2URDF.Export\">"
                + "<angularDeflection>0.75</angularDeflection>"
                + "<linearDeflection>0.5</linearDeflection>"
                + "<scale>1</scale>"
                + "</MeshingOptions>";
            var ser = new DataContractSerializer(typeof(MeshingOptions));
            using (var stream = new MemoryStream(Encoding.UTF8.GetBytes(xml)))
            {
                var options = (MeshingOptions)ser.ReadObject(stream);

                Assert.Equal(0.75, options.angularDeflection);
                Assert.Equal(0.5, options.linearDeflection);
                Assert.Equal(MeshingBackend.Isotropic, options.backend);
            }
        }
    }
}
