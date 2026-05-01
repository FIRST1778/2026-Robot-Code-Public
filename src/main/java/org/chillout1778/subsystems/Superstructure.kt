package org.chillout1778.subsystems

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.Robot
import org.chillout1778.Util
import kotlin.math.min

object Superstructure : SubsystemBase() {
    var inputs = Controls.controls

    private var currentAimTarget: Translation2d = Constants.Field.BLUE_HUB

    var pitMode = false

    override fun periodic() {
        // DISABLE SUPERSTRUCTURE IF TUNING
        if (Constants.TUNING_MODE) {
            return
        }

        inputs = Controls.controls

        var passing = false

        Turret.updateDynamicOffset(if (inputs.wantTurretZero) inputs.zeroStickAxis else 0.0)

        // Rumble Left if Intake current is > 90% of the limit, Rumble Right if Spindexer current is > 90% of the limit
        Controls.operatorRumble(
            if (Intake.rollerSupplyCurrent > Constants.CurrentLimits.INTAKE_ROLLERS * 0.9) 1.0 else 0.0,
            if (Indexer.spindexerSupplyCurrent > Constants.CurrentLimits.SPINDEXER * 0.9) 1.0 else 0.0
        )

        currentAimTarget = if (Robot.isRedAlliance) {
            // If we're in the red alliance zone
            if (Robot.swervePose.x > Constants.Field.RED_X_PASS_THRESHOLD || Robot.autoIsRunning) {
                passing = false
                Constants.Field.RED_HUB
            } else {
                passing = true
                Translation2d(Constants.Field.RED_PASS_X, Robot.swervePose.y)
            }
        } else {
            //  If we're in the blue alliance zone
            if (Robot.swervePose.x < Constants.Field.BLUE_X_PASS_THRESHOLD || Robot.autoIsRunning) {
                passing = false
                Constants.Field.BLUE_HUB
            } else {
                passing = true
                Translation2d(Constants.Field.BLUE_PASS_X, Robot.swervePose.y)
            }
        }

        val distanceAndAngleToTarget = Util.getDistanceToTargetTranslation(currentAimTarget)

        // Rotate turret to aim at hub
        if (!pitMode) {
            Turret.desiredAngle = distanceAndAngleToTarget.second
        } else {
            Turret.desiredAngle = 0.0 // Aim turret forward in pit mode
        }

        // If we don't want to shoot, just put the hood down
        if (!inputs.wantShoot) {
            Shooter.moveHoodTo(Constants.Shooter.HOOD_TRENCH_ANGLE)
            Shooter.flywheelVelocity = min(Constants.Shooter.speedLUT.get(distanceAndAngleToTarget.first), Constants.Shooter.SHOOTER_FLYWHEEL_IDLE_RPM_MAX)
            Indexer.state = Indexer.IndexerState.STOP
        } else {
            // We want to shoot, so use our LUTs to determine flywheel & hood setpoints

            if (!pitMode) {
                Shooter.flywheelVelocity = Constants.Shooter.speedLUT.get(distanceAndAngleToTarget.first)
                Shooter.moveHoodTo(Constants.Shooter.angleLUT.get(distanceAndAngleToTarget.first))
            } else {
                Shooter.flywheelVelocity = 600.0
                Shooter.moveHoodTo(0.0)
                passing = false
            }

            if (passing) {
                if (Turret.atSetpoint && (Robot.swervePose.y !in Constants.Field.HUB_AVOIDANCE_RANGE)) {
                    Indexer.state = Indexer.IndexerState.START
                } else {
                    Indexer.state = Indexer.IndexerState.STOP
                }
                Indexer.state = Indexer.IndexerState.START
            } else {
                // If turret and flywheel and hood are all at their setpoints, we can start shooting
                if (Turret.atSetpoint && Shooter.atSetpoint) {
                    Indexer.state = Indexer.IndexerState.START
                } else {
                    // If we're not at a setpoint yet, don't shoot balls
                    Indexer.state = Indexer.IndexerState.STOP
                }
                Indexer.state = Indexer.IndexerState.START
            }
        }
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addBooleanProperty("Pit mode", { pitMode }, { pitMode = it })
        builder.addBooleanProperty("LUT tuning mode", { Constants.TUNING_MODE }, { Constants.TUNING_MODE = it })
        builder.addBooleanProperty("Tuning passing", { Constants.TUNING_PASSING }, { Constants.TUNING_PASSING = it })

        builder.addDoubleProperty("Controls Forward Drive", {inputs.forward}, {})
        builder.addDoubleProperty("Controls Left Drive", {inputs.left}, {})
        builder.addDoubleProperty("Controls Rotation", {inputs.rotation}, {})

        builder.addBooleanProperty("Controls Want Shoot", {inputs.wantShoot}, {})
        builder.addBooleanProperty("Controls Want Intake Shoot Retract", {inputs.wantIntakeShootRetract}, {})
        builder.addBooleanProperty("Controls Want Reverse Spindexer", { inputs.wantReverseSpindexer }, {})
        builder.addBooleanProperty("Controls Want Flywheel Reverse", { inputs.wantFlywheelReverse }, {})

        builder.addBooleanProperty("Controls Want Intake Out", { inputs.wantIntakeOut }, {})
        builder.addBooleanProperty("Controls Want Intake Spinning", { inputs.wantIntakeSpinning }, {})
        builder.addBooleanProperty("Controls Want Intake Reversed", { inputs.wantIntakeReversed }, {})

        builder.addDoubleProperty("Controls Zero Stick Axis", { inputs.zeroStickAxis }, {})
        builder.addBooleanProperty("Controls Want Turret Zero", { inputs.wantTurretZero }, {})
    }
}