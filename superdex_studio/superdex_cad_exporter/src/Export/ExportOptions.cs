/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.Serialization;

using Meshing;

namespace CADRobotExporter.Export
{
    [Serializable]
    public class ExporterMeshingOptions
    {
        public string meshTagExtension;
        public bool exportCollision;
        public string collisionMeshTagExtension;
        public MeshingOptions visualMeshingOptions;
        public MeshingOptions collisionMeshingOptions;

        public ExporterMeshingOptions()
        {
            visualMeshingOptions = new MeshingOptions();
            collisionMeshingOptions = new MeshingOptions();
        }

        public ExporterMeshingOptions Clone()
        {
            return new ExporterMeshingOptions
            {
                meshTagExtension = meshTagExtension,
                exportCollision = exportCollision,
                collisionMeshTagExtension = collisionMeshTagExtension,
                visualMeshingOptions = visualMeshingOptions,
                collisionMeshingOptions = collisionMeshingOptions,
            };
        }

        /// <summary>
        /// The settings to mesh one link with: its own, if it is marked as overriding, and otherwise
        /// these robot-wide ones.
        /// </summary>
        public ExporterMeshingOptions ResolveFor(RobotDescription.Link link)
        {
            if (link == null || !link.meshingOverride)
            {
                return this;
            }

            ExporterMeshingOptions resolved = Clone();
            resolved.visualMeshingOptions = link.visualMeshingOptions ?? visualMeshingOptions;
            resolved.collisionMeshingOptions =
                link.collisionMeshingOptions ?? collisionMeshingOptions;
            return resolved;
        }
    }

    [Serializable]
    [DataContract(Namespace = "http://schemas.datacontract.org/2004/07/SW2URDF.Export")]
    [SuppressMessage(
        "Microsoft.Usage",
        "CA2239:ProvideDeserializationMethodsForOptionalFields",
        Justification = "The optional fields are seeded from [OnDeserializing], which runs before "
            + "the data is read so values actually present still win. The [OnDeserialized] method "
            + "this rule asks for runs afterwards and would overwrite them, and could not tell an "
            + "absent field from one explicitly saved as the enum's zero value.")]
    public class MeshingOptions
    {
        [DataMember]
        public double linearDeflection;
        [DataMember]
        public double angularDeflection;
        [DataMember]
        public double scale;

        /// <summary>Face tessellation backend.</summary>
        [OptionalField]
        [DataMember]
        public MeshingBackend backend = MeshingBackend.Isotropic;

        /// <summary>Edge sampling strategy. Ignored by <see cref="MeshingBackend.Delabella"/>.</summary>
        [OptionalField]
        [DataMember]
        public EdgeSampling edgeSampling = EdgeSampling.Adaptive;

        /// <summary>
        /// Target triangle edge length [mm]. Zero means derive it from
        /// <see cref="targetEdgeLengthFraction"/> instead. Ignored by
        /// <see cref="MeshingBackend.Delabella"/>.
        /// </summary>
        [OptionalField]
        [DataMember]
        public double targetEdgeLength = 0.0;

        /// <summary>
        /// Fraction of the bounding-box diagonal used as the target edge length when
        /// <see cref="targetEdgeLength"/> is zero. Ignored by <see cref="MeshingBackend.Delabella"/>.
        /// </summary>
        [OptionalField]
        [DataMember]
        public double targetEdgeLengthFraction = 0.02;

        /// <summary>
        /// Sets the optional fields before deserialization populates whatever the data actually
        /// carries.
        /// </summary>
        [OnDeserializing]
        private void OnDeserializing(StreamingContext context)
        {
            backend = MeshingBackend.Isotropic;
            edgeSampling = EdgeSampling.Adaptive;
            targetEdgeLength = 0.0;
            targetEdgeLengthFraction = 0.02;
        }

        public MeshingOptions Clone()
        {
            return new MeshingOptions
            {
                linearDeflection = linearDeflection,
                angularDeflection = angularDeflection,
                scale = scale,
                backend = backend,
                edgeSampling = edgeSampling,
                targetEdgeLength = targetEdgeLength,
                targetEdgeLengthFraction = targetEdgeLengthFraction,
            };
        }
    }
}
