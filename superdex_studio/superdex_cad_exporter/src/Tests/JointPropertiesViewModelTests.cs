/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

using CADRobotExporter.RobotDescription;
using CADRobotExporter.UI.ViewModels;
using Xunit;

namespace CADRobotExporter.Tests
{
    /// <summary>
    /// Covers the joint properties page mapping, in particular the angle unit policy: origin
    /// rotations are always angular, but limits and velocity are only angular on a revolute joint.
    /// </summary>
    public class JointPropertiesViewModelTests
    {
        private const double HalfPi = 1.5707963267948966;

        private static Joint RevoluteJoint()
        {
            Joint joint = new Joint();
            joint.Name = "elbow";
            joint.Type = "revolute";
            joint.Origin.X = 1.0;
            joint.Origin.Y = 2.0;
            joint.Origin.Z = 3.0;
            joint.Origin.Roll = HalfPi;
            joint.Origin.Pitch = 0.0;
            joint.Origin.Yaw = 0.0;
            joint.Axis.X = 0.0;
            joint.Axis.Y = 0.0;
            joint.Axis.Z = 1.0;
            joint.Limit.Lower = -HalfPi;
            joint.Limit.Upper = HalfPi;
            joint.Limit.Effort = 10.0;
            joint.Limit.Velocity = HalfPi;
            return joint;
        }

        [Fact]
        public void OriginRotations_ConvertToDegreesWhenRequested()
        {
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(RevoluteJoint(), useDegrees: true);

            Assert.Equal(90.0, double.Parse(properties.Origin.Roll), 6);
        }

        [Fact]
        public void OriginRotations_StayInRadiansWhenNotRequested()
        {
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(RevoluteJoint(), useDegrees: false);

            Assert.Equal(HalfPi, double.Parse(properties.Origin.Roll), 6);
        }

        [Fact]
        public void OriginTranslations_AreNeverConverted()
        {
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(RevoluteJoint(), useDegrees: true);

            Assert.Equal("1", properties.Origin.X);
        }

        [Fact]
        public void RevoluteLimits_ConvertToDegrees()
        {
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(RevoluteJoint(), useDegrees: true);

            Assert.Equal(90.0m, decimal.Round(properties.LimitUpper, 4));
            Assert.Equal(90.0, double.Parse(properties.LimitVelocity), 6);
        }

        /// <summary>
        /// A prismatic joint's limits are metres. Converting them would silently scale the travel
        /// by 57 whenever the degrees checkbox is on.
        /// </summary>
        [Fact]
        public void PrismaticLimits_AreNeverConvertedEvenInDegreesMode()
        {
            Joint joint = RevoluteJoint();
            joint.Type = "prismatic";
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(joint, useDegrees: true);

            Assert.Equal(HalfPi, (double)properties.LimitUpper, 6);
            Assert.Equal(HalfPi, double.Parse(properties.LimitVelocity), 6);
        }

        /// <summary>
        /// A prismatic joint's origin rotations are still angles, so they do convert.
        /// </summary>
        [Fact]
        public void PrismaticOriginRotations_StillConvert()
        {
            Joint joint = RevoluteJoint();
            joint.Type = "prismatic";
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(joint, useDegrees: true);

            Assert.Equal(90.0, double.Parse(properties.Origin.Roll), 6);
        }

        [Theory]
        [InlineData(true)]
        [InlineData(false)]
        public void RevoluteJoint_RoundTripsThroughDisplayUnits(bool useDegrees)
        {
            Joint source = RevoluteJoint();
            JointPropertiesViewModel properties = new JointPropertiesViewModel();
            properties.LoadFrom(source, useDegrees);

            Joint target = new Joint();
            properties.SaveTo(target, useDegrees);

            Assert.Equal(source.Origin.Roll, target.Origin.Roll, 9);
            Assert.Equal(source.Limit.Lower, target.Limit.Lower, 6);
            Assert.Equal(source.Limit.Upper, target.Limit.Upper, 6);
            Assert.Equal(source.Limit.Velocity, target.Limit.Velocity, 6);
            Assert.Equal(source.Limit.Effort, target.Limit.Effort, 9);
        }

        [Fact]
        public void FixedJoint_DisallowsMotionAndHidesAxis()
        {
            Joint joint = RevoluteJoint();
            joint.Type = "fixed";
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(joint, useDegrees: false);

            Assert.False(properties.AllowsMotion);
            Assert.Equal(string.Empty, properties.Axis.X);
        }

        [Fact]
        public void NullJoint_ReportsNoJointAndDefaultLabels()
        {
            JointPropertiesViewModel properties = new JointPropertiesViewModel();

            properties.LoadFrom(null, useDegrees: true);

            Assert.False(properties.HasJoint);
            Assert.Equal("Lower", properties.Labels.Lower);
            Assert.Equal("Damping", properties.Labels.Damping);
        }

        [Theory]
        [InlineData("revolute", true, "Lower (deg)", "Damping (N-m-s/deg)")]
        [InlineData("revolute", false, "Lower (rad)", "Damping (N-m-s/rad)")]
        [InlineData("continuous", true, "Lower (deg)", "Damping (N-m-s/deg)")]
        [InlineData("prismatic", true, "Lower (m)", "Damping (N-s/m)")]
        [InlineData("fixed", true, "Lower", "Damping")]
        public void Labels_CarryTheUnitsForTheJointType(
            string jointType, bool useDegrees, string expectedLower, string expectedDamping)
        {
            JointLabels labels = JointLabels.For(jointType, useDegrees);

            Assert.Equal(expectedLower, labels.Lower);
            Assert.Equal(expectedDamping, labels.Damping);
        }

        [Theory]
        [InlineData("revolute", true, 5)]
        [InlineData("revolute", false, 0.0872665)]
        [InlineData("prismatic", true, 0.1)]
        [InlineData("fixed", false, 0.1)]
        public void LimitIncrement_MatchesTheJointTypeAndUnits(
            string jointType, bool useDegrees, decimal expected)
        {
            Assert.Equal(expected, JointAngles.LimitIncrementFor(jointType, useDegrees));
        }

        /// <summary>
        /// Blank effort and velocity on an optional joint must leave the limit unset rather than
        /// writing zeros, which would pin the joint shut.
        /// </summary>
        [Fact]
        public void BlankOptionalLimit_IsLeftUnwritten()
        {
            Joint joint = new Joint();
            joint.Type = "fixed";
            JointPropertiesViewModel properties = new JointPropertiesViewModel();
            properties.Clear();
            properties.Type = "fixed";
            properties.LimitLower = 5m;
            properties.LimitUpper = 6m;

            properties.SaveTo(joint, useDegrees: false);

            Assert.False(joint.Limit.IsLowerSet());
            Assert.False(joint.Limit.IsUpperSet());
        }

        [Fact]
        public void RequiredLimit_IsWrittenEvenWhenEffortAndVelocityAreBlank()
        {
            Joint joint = new Joint();
            JointPropertiesViewModel properties = new JointPropertiesViewModel();
            properties.Clear();
            properties.Type = "revolute";
            properties.LimitLower = -1m;
            properties.LimitUpper = 1m;

            properties.SaveTo(joint, useDegrees: false);

            Assert.True(joint.Limit.IsLowerSet());
            Assert.Equal(-1.0, joint.Limit.Lower, 9);
        }

        [Fact]
        public void BlankDynamics_UnsetsRatherThanWritingZeros()
        {
            Joint joint = RevoluteJoint();
            joint.Dynamics.Damping = 3.0;
            joint.Dynamics.Friction = 4.0;
            JointPropertiesViewModel properties = new JointPropertiesViewModel();
            properties.Clear();
            properties.Type = "revolute";

            properties.SaveTo(joint, useDegrees: false);

            Assert.False(joint.Dynamics.IsDampingSet());
            Assert.False(joint.Dynamics.IsFrictionSet());
        }
    }
}
