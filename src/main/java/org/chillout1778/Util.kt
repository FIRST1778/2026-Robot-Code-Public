package org.chillout1778

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import org.chillout1778.subsystems.Swerve
import kotlin.math.abs
import kotlin.math.atan2

object Util {

    data class Zone2d(val corner1: Translation2d, val corner2: Translation2d)

    fun poseInZone (input: Pose2d, zone: Zone2d) = (
            (input.x in zone.corner1.x..zone.corner2.x)
                    && (input.y in zone.corner1.y..zone.corner2.y)
            )

    open class Toggle<T>(val reference: () -> Boolean, val states: List<T>) {
        private var currentIndex = 0
        var lastReference = false

        fun get(): T {
            if (reference() && !lastReference) { // If we just pressed the button, and it wasn't pressed the last time get() was called
                lastReference = true
                currentIndex++
                if (currentIndex == states.size) {
                    currentIndex = 0
                }
            }
            else if (!reference()) {
                lastReference = false
            }

            return states[currentIndex]
        }
    }

    class BooleanToggle(reference: () -> Boolean, invert: Boolean): Toggle<Boolean>(reference, listOf(!invert, invert)) {
        constructor(reference: () -> Boolean) : this(reference, false)
    }

    fun getDistanceToTargetTranslation(target: Translation2d) : Pair<Double, Double> {
        var realTarget = target

        val robotSpeeds = Swerve.state.Speeds
        val fieldCentricSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotSpeeds.vxMetersPerSecond,
            robotSpeeds.vyMetersPerSecond,
            robotSpeeds.omegaRadiansPerSecond,
            Robot.swervePose.rotation)

        if (Constants.Turret.USE_SOTM) {
            realTarget = Translation2d(target.x - (fieldCentricSpeeds.vxMetersPerSecond * Constants.Turret.SOTM_TRANSLATIONAL_SCALAR),
                target.y - (fieldCentricSpeeds.vyMetersPerSecond * Constants.Turret.SOTM_TRANSLATIONAL_SCALAR))
        }
        
        val swerveTurretTransform = Transform2d(Constants.Vision.SWERVE_TO_TURRET_TRANSFORM.x, Constants.Vision.SWERVE_TO_TURRET_TRANSFORM.y, Constants.Vision.SWERVE_TO_TURRET_TRANSFORM.rotation.toRotation2d())
        val shooterPose = Robot.swervePose.transformBy(swerveTurretTransform)

        val differenceX = realTarget.x - shooterPose.x
        val differenceY = realTarget.y - shooterPose.y

        val angletoHub = atan2(differenceY, differenceX)

        return Pair(
            shooterPose.translation.getDistance(
                realTarget),

            // field-relative angle from robot to hub minus robot heading, with rotational compensation
            (angletoHub - Robot.swervePose.rotation.radians) - (if(Constants.Turret.USE_SOTM) (fieldCentricSpeeds.omegaRadiansPerSecond * Constants.Turret.SOTM_ROTATIONAL_SCALAR) else 0.0)
        )
    }
}