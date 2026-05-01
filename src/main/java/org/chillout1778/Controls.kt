package org.chillout1778

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller

object Controls {
    private val driverController = CommandPS5Controller(0)
    private val operatorController = CommandPS5Controller(1)

    data class Inputs(
        // NOTE: Some values are vars so that autos can change them

        // Driving
        var forward: Double = 0.0,
        var left: Double = 0.0,
        var rotation: Double = 0.0,
        var wantSwerveBrake: Boolean = false,

        // Shooting
        var wantShoot: Boolean = false,
        var wantIntakeShootRetract: Boolean = false,
        var wantReverseSpindexer: Boolean = false,
        var wantFlywheelReverse: Boolean = false,

        // Intaking
        var wantIntakeOut: Boolean = false,
        var wantIntakeSpinning: Boolean = false,
        var wantIntakeReversed: Boolean = false,

        // Turret
        val zeroStickAxis: Double = 0.0,
        val wantTurretZero: Boolean = false,
        var wantAutomaticTurretZero: Boolean = false,
        val wantTurretHoldPosition: Boolean = false
    )

    var autoControls = Inputs()

    val controls: Inputs get() {
        return if (Robot.isAutonomous) autoControls
        else Inputs(
            // Driving
            forward = -driverController.leftY,
            left = -driverController.leftX,
            rotation = -driverController.rightX,
            wantSwerveBrake = driverController.hid.l3Button,

            // Shooting
            wantShoot = driverController.hid.r2Button,
            wantIntakeShootRetract = driverController.hid.r1Button || operatorController.hid.l2Button,
            wantReverseSpindexer = operatorController.hid.circleButton || driverController.hid.crossButton,
            wantFlywheelReverse = operatorController.hid.triangleButton,

            // Intaking
            wantIntakeOut = operatorController.hid.r2Button,
            wantIntakeSpinning = operatorController.hid.r1Button,
            wantIntakeReversed = operatorController.hid.l1Button,

            // Turret
            zeroStickAxis = operatorController.leftX,
            wantTurretZero = operatorController.hid.l3Button,
            wantAutomaticTurretZero = operatorController.hid.crossButton || driverController.hid.circleButton,
            wantTurretHoldPosition = operatorController.hid.squareButton
        )
    }

    fun operatorRumble(left: Double, right: Double) {
        operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, left)
        operatorController.setRumble(GenericHID.RumbleType.kRightRumble, right)
    }

}