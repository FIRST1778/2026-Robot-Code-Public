// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.chillout1778

import choreo.auto.AutoFactory
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.*
import org.chillout1778.commands.TeleOpDriveCommand
import org.chillout1778.subsystems.*
import org.chillout1778.subsystems.Indexer.ballTunnelPid
import org.chillout1778.subsystems.Indexer.spindexerPid
import org.chillout1778.subsystems.Intake.extensionPid
import org.chillout1778.subsystems.Shooter.flywheelPid
import org.chillout1778.subsystems.Shooter.hoodPid
import org.chillout1778.subsystems.Turret.turretPid

object Robot : TimedRobot() {

    val isRedAlliance get() = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    var isRedAllianceAutoWinner = false

    var swervePose = Pose2d(0.0, 0.0, Rotation2d(0.0))

    var matchTimer = Timer()

    val autoFactory = AutoFactory(
        { swervePose },
        {pose: Pose2d -> Swerve.resetPose(pose)},
        Swerve::followTrajectory,
        true,
        Swerve
    )

    var selectedAutoCommand: Command = InstantCommand()

    val autoChooser: SendableChooser<Command> = SendableChooser()

    var autoIsRunning = false
    
    override fun robotInit() {
        Shuffleboard.getTab("Swerve").add(Swerve)
        Shuffleboard.getTab("Subsystems").add(Intake)
        Shuffleboard.getTab("Subsystems").add(Indexer)
        Shuffleboard.getTab("Subsystems").add(Shooter)
        Shuffleboard.getTab("Subsystems").add(Superstructure)
        Shuffleboard.getTab("Subsystems").add(Turret)
        Shuffleboard.getTab("Subsystems").add(Vision)
        Shuffleboard.getTab("Subsystems").add(Lights)
        Shuffleboard.getTab("Subsystems").add("Flywheel PID", flywheelPid)
        Shuffleboard.getTab("Subsystems").add("Spindexer PID", spindexerPid)
        Shuffleboard.getTab("Subsystems").add("Ball tunnel PID", ballTunnelPid)
        Shuffleboard.getTab("Subsystems").add("Hood PID", hoodPid)
        Shuffleboard.getTab("Subsystems").add("Intake extension PID", extensionPid)
        Shuffleboard.getTab("Subsystems").add("Turret PID", turretPid)

        Shuffleboard.getTab("PDH").add(pdh)

        // Add autos to auto chooser
        autoChooser.addOption("No auto", InstantCommand())
        autoChooser.addOption("Right Trench Auto", AutoContainer.rightTrenchAuto)
        autoChooser.addOption("Left Trench Auto", AutoContainer.leftTrenchAuto)
        autoChooser.addOption("Left Depot Auto", AutoContainer.leftDepotAuto)
        //  autoChooser.addOption("Bump Auto", AutoContainer.bumpAuto)

        autoChooser.onChange {
            selectedAutoCommand = it
        }

        Shuffleboard.getTab("Robot").add(autoChooser)
    }

    object AutoContainer {
        private val clearAutoControlsCommand get() = InstantCommand({ Controls.autoControls = Controls.Inputs() })
        private val intakeExtendCommand get() = InstantCommand({ Controls.autoControls.wantIntakeOut = true})
        private val intakeRetractCommand get() = InstantCommand({ Controls.autoControls.wantIntakeOut = false})
        private val startIntakeCommand get() = InstantCommand({ Controls.autoControls.wantIntakeSpinning = true})
        private val stopIntakeCommand get() = InstantCommand({ Controls.autoControls.wantIntakeSpinning = false})
        private val startShootCommand get() = Commands.parallel(
            InstantCommand({ Controls.autoControls.wantShoot = true}),
            Commands.sequence(
                InstantCommand({ Controls.autoControls.wantAutomaticTurretZero = true }),
                WaitCommand(0.5),
                InstantCommand({ Controls.autoControls.wantAutomaticTurretZero = false })
            )
        )
        private val stopShootCommand get() = InstantCommand({ Controls.autoControls.wantShoot = false})
        private val oscillateInCommand get() = InstantCommand({Controls.autoControls.wantIntakeShootRetract = true})
        private val oscillateOutCommand get() = InstantCommand({Controls.autoControls.wantIntakeShootRetract = false})
        private val oscillateIntake get() = Commands.sequence(
            oscillateInCommand,
            WaitCommand(0.25),
            oscillateOutCommand,
            WaitCommand(0.25),
        )
        private val stopDrivetrainCommand get() = InstantCommand({
            Swerve.setControl(SwerveRequest.ApplyFieldSpeeds())
        })

        val bumpAuto get() = Commands.sequence(
            clearAutoControlsCommand,
            autoFactory.resetOdometry("BumpShartTripleWhammyP1"),
            intakeExtendCommand,
            WaitCommand(0.1),
            startIntakeCommand,
            autoFactory.trajectoryCmd("BumpShartTripleWhammyP1"),
            stopDrivetrainCommand,
            stopIntakeCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopShootCommand,
            startIntakeCommand,
            autoFactory.trajectoryCmd("BumpShartTripleWhammyP2"),
            stopDrivetrainCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopIntakeCommand,
            stopShootCommand,
            autoFactory.trajectoryCmd("BumpShartTripleWhammyP3"),
            stopDrivetrainCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopIntakeCommand,
            stopShootCommand
        )

        val rightTrenchAuto get() = Commands.sequence(
            clearAutoControlsCommand,
            autoFactory.resetOdometry("FuelMaxxingUltimateAllianceDoubleWhammyMogP1"),
            intakeExtendCommand,
            WaitCommand(0.1),
            startIntakeCommand,
            autoFactory.trajectoryCmd("FuelMaxxingUltimateAllianceDoubleWhammyMogP1"),
            stopDrivetrainCommand,
            stopIntakeCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopShootCommand,
            startIntakeCommand,
            autoFactory.trajectoryCmd("FuelMaxxingUltimateAllianceDoubleWhammyMogP2"),
            stopDrivetrainCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopIntakeCommand,
            stopShootCommand
            )

        val leftTrenchAuto get() = Commands.sequence(
            clearAutoControlsCommand,
            autoFactory.resetOdometry("StephFlurrysRevenge"),
            intakeExtendCommand,
            startIntakeCommand,
            autoFactory.trajectoryCmd("StephFlurrysRevenge"),
            stopDrivetrainCommand,
            stopIntakeCommand,
            startShootCommand,
            WaitCommand(0.5),
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopShootCommand,
            startIntakeCommand,
            autoFactory.trajectoryCmd("StephFlurrysRevengeTheSequel"),
            stopDrivetrainCommand,
            startShootCommand,
            WaitCommand(0.5),
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
        )

        val leftDepotAuto get() = Commands.sequence(
            clearAutoControlsCommand,
            autoFactory.resetOdometry("DepotRizzler"),
            intakeExtendCommand,
            startIntakeCommand,
            autoFactory.trajectoryCmd("DepotRizzler"),
            stopDrivetrainCommand,
            stopIntakeCommand,
            startShootCommand,
            WaitCommand(2.0),
            oscillateIntake,
            oscillateIntake,
            oscillateIntake,
            stopShootCommand,
            intakeRetractCommand
        )

        val passingAuto get() = Commands.sequence(
            clearAutoControlsCommand,
            autoFactory.resetOdometry("RIPMyHoardingGrandmaP1"),
            intakeExtendCommand,
            WaitCommand(0.1),
            startIntakeCommand,
            autoFactory.trajectoryCmd("RIPMyHoardingGrandmaP1"),
            stopDrivetrainCommand,
            stopIntakeCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopShootCommand,
            startIntakeCommand,
            autoFactory.trajectoryCmd("RIPMyHoardingGrandmaP2"),
            stopDrivetrainCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopIntakeCommand,
            stopShootCommand,
            autoFactory.trajectoryCmd("RIPMyHoardingGrandmaP3"),
            stopDrivetrainCommand,
            startShootCommand,
            oscillateIntake,
            oscillateIntake,
            oscillateInCommand,
            WaitCommand(1.25),
            oscillateOutCommand,
            stopIntakeCommand,
            stopShootCommand
        )

    }

    var pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        swervePose = Pose2d(0.0, 0.0, Rotation2d(0.0))
        val driveState = Swerve.state
        swervePose = driveState.Pose

        if (Constants.TUNING_MODE) {
            val distanceAndAngleToTarget = Util.getDistanceToTargetTranslation(
                if (Constants.TUNING_PASSING)
                            Translation2d(if (isRedAlliance) Constants.Field.RED_PASS_X else Constants.Field.BLUE_PASS_X,
                                        Robot.swervePose.y)
                        else if (isRedAlliance) Constants.Field.RED_HUB else Constants.Field.BLUE_HUB
            )
            Turret.desiredAngle = distanceAndAngleToTarget.second
            if (Controls.controls.wantReverseSpindexer) {
                Indexer.state = Indexer.IndexerState.START
            } else {
                Indexer.state = Indexer.IndexerState.STOP
            }
        }
    }

    override fun teleopInit() {
        Swerve.defaultCommand = TeleOpDriveCommand(Controls::controls)
        autoIsRunning = false

        matchTimer.reset()
        matchTimer.start()
    }

    override fun teleopPeriodic() {
        val gameData = DriverStation.getGameSpecificMessage()
        if (gameData != null && gameData.length > 0) {
            when (gameData) {
                "R" ->
                    isRedAllianceAutoWinner = true
                else ->
                    isRedAllianceAutoWinner = false
            }
        }
    }


    fun isOurHubEnabledNow(): Boolean {
        // The FMS has a method Timer.getMatchTime() which is kinda meant for the job.
        // but also, it doesn't work during practice. nor does it guarantee any precision

        // Get the current shift number
        val shift = allianceShiftNumber
        // If we're in auto/endgame, we are active
        if (shift == -1) return true

        // if we are red and won, or are blue and won
        val isOurAllianceAutoWinner = !(isRedAllianceAutoWinner.xor(isRedAlliance))

        // if we won the auto, and the current shift is even
        // or if we lost the auto, and our current shift is uneven
        return (isOurAllianceAutoWinner && (shift % 2) == 0) || (!isOurAllianceAutoWinner && (shift % 2) == 1)
    }


    // Returns the current shift number, starting from 1. value of -1 is reported during auto/ endgame
    val allianceShiftNumber: Int get() =
        if (matchTimer.get() < 10.0 || matchTimer.get() > 110.0) -1
        else (((matchTimer.get() - 10.0) / 25.0).toInt() + 1)

    override fun teleopExit() {
        Swerve.removeDefaultCommand()
    }

    override fun autonomousInit() {
        Controls.autoControls = Controls.Inputs()

        CommandScheduler.getInstance().schedule(selectedAutoCommand)
        autoIsRunning = true
    }

    override fun autonomousExit() {
        Controls.autoControls = Controls.Inputs()
        autoIsRunning = false
    }
}
