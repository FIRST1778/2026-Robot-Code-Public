package org.chillout1778.subsystems

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

object Turret : SubsystemBase() {

    val pivotMotor = TalonFX(Constants.CanIds.TURRET_PIVOT_MOTOR)

    private val encoder1 = CANcoder(Constants.CanIds.TURRET_CANCODER_1)
    private val encoder2 = CANcoder(Constants.CanIds.TURRET_CANCODER_2) // NOTE: Not used in code

    private var dynamicOffset = 0.0

    var automaticOffset = 0.0

    var desiredAngle = 0.0 // radians

    var turretPid = Constants.Turret.TURRET_PID_COEFFICIENTS.createPid()

    var atSetpoint = false

    var wantDriverAutomaticTurretPrevious = false

    init {
        pivotMotor.configurator.apply(Constants.Turret.TURRET_PIVOT_CONFIG)

        pivotMotor.setPosition(0.0)

        turretPid.setTolerance(toRadians(5.0))

        encoder1.optimizeBusUtilization()
        encoder2.optimizeBusUtilization()
        pivotMotor.optimizeBusUtilization()

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            pivotMotor.position, pivotMotor.velocity,
            encoder1.position, encoder2.position,
            encoder1.absolutePosition, encoder2.absolutePosition)
        BaseStatusSignal.setUpdateFrequencyForAll(10.0,
            pivotMotor.supplyVoltage, pivotMotor.supplyCurrent)
    }

    override fun periodic() {
        if (pivotMotor.velocity.valueAsDouble < Constants.Turret.MAXIMUM_ABSOLUTE_ENCODER_VELOCITY &&
            (Controls.controls.wantAutomaticTurretZero ||
                    (Controls.controls.wantShoot && !wantDriverAutomaticTurretPrevious))) {
            automaticOffset +=
                (getAbsolutePositionUsingAbsoluteEncoderAssumingTheRelativeEncoderIsntExtremelyFarOff() -
                        getTurretOrientation())
        }

        wantDriverAutomaticTurretPrevious = Controls.controls.wantShoot

        turretPid.setpoint = if (Controls.controls.wantTurretHoldPosition) getTurretOrientation() else findTurretSetpoint(desiredAngle)

        val current = getTurretOrientation()

        pivotMotor.setVoltage((turretPid.calculate(current)
                + Constants.Turret.TURRET_PID_COEFFICIENTS.kS * sign(turretPid.setpoint - current)
            ).coerceIn(-5.5, 5.5))

        // We're at the setpoint if we're within 2 degrees
        atSetpoint = turretPid.atSetpoint()
    }

    fun getAbsolutePositionUsingAbsoluteEncoderAssumingTheRelativeEncoderIsntExtremelyFarOff() : Double {
        // Absolute encoder heading
        val absoluteEncoderPosition = encoder1.absolutePosition.valueAsDouble - Constants.Turret.BIG_ENCODER_ZERO

        // Absolute encoder heading in radians
        val absoluteEncoderRadians = absoluteEncoderPosition * (2 * PI) // -PI to PI

        val potentialTurretAngles = mutableListOf(absoluteEncoderRadians / Constants.Turret.ENCODER_TURRET_RATIO)

        for (i in 1..7) { // TODO: Check this number if it is giving incorrect values in some positions, or if the ENCODER_TURRET_RATIO changes
            potentialTurretAngles.add(((absoluteEncoderRadians + (2 * PI * i)) / Constants.Turret.ENCODER_TURRET_RATIO))
            potentialTurretAngles.add(((absoluteEncoderRadians - (2 * PI * i)) / Constants.Turret.ENCODER_TURRET_RATIO))
        }

        potentialTurretAngles.sort()

        val relativeEncoderTurretHeading = getTurretOrientation()

        var selectedAngle = potentialTurretAngles[0]
        var selectedIndex = 0
        var matchFound = false

        while (!matchFound) {
            if (abs(potentialTurretAngles[selectedIndex + 1] - relativeEncoderTurretHeading) < abs(selectedAngle - relativeEncoderTurretHeading)) {
                selectedIndex++
                selectedAngle = potentialTurretAngles[selectedIndex]
            } else {
                matchFound = true
            }
        }

        return selectedAngle
    }


    // NOTE: This intentionally does NOT normalize the angle! This is because we need to track our amount of rotation in
    // each direction, even if we pass the 'wrap' point. We need to track this because we have limited wire length
    // and need to go back the other way if we're to the end of our range.
    fun getTurretOrientation() : Double {
        // Radians from the motor's 0 position
        val motorRevolutions = pivotMotor.position.valueAsDouble
        val turretRevolutions = motorRevolutions / Constants.Turret.MOTOR_TO_TURRET_RATIO
        val turretRadians = turretRevolutions * 2 * PI

        val actualRadians = turretRadians + dynamicOffset + automaticOffset

        return actualRadians
    }

    // This function clamps the supplied angle into the turrets operating range.
    fun findTurretSetpoint(desiredAngle: Double): Double {
        var selectedSetpoint = desiredAngle
        while (selectedSetpoint !in Constants.Turret.TURRET_OPERATING_RANGE) {
            if (selectedSetpoint > Constants.Turret.TURRET_OPERATING_RANGE.endInclusive) {
                selectedSetpoint -= 2 * PI
            } else if (selectedSetpoint < Constants.Turret.TURRET_OPERATING_RANGE.start) {
                selectedSetpoint += 2 * PI
            }
        }

        return selectedSetpoint
    }

    // Function to rezero the turret during match
    fun updateDynamicOffset (input: Double) {
        dynamicOffset += input * Constants.Turret.DYNAMIC_OFFSET_CHANGE_SPEED
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Turret Orientation", { getTurretOrientation() }, {})
        builder.addDoubleProperty("Turret Setpoint", { desiredAngle }, { desiredAngle = it })
        builder.addDoubleProperty("Turret kV", { Constants.Turret.TURRET_PID_COEFFICIENTS.kS }, {Constants.Turret.TURRET_PID_COEFFICIENTS.kS= it})
        builder.addDoubleProperty("Turret error", { turretPid.error }, {})
        builder.addDoubleProperty("Dynamic Turret Offset", { dynamicOffset }, {})
        builder.addDoubleProperty("Automatic turret offset", { automaticOffset }, {})
        builder.addDoubleProperty("Turret motor velocity", { pivotMotor.velocity.valueAsDouble }, {})
        builder.addDoubleProperty("Absolute encoder estimate", { getAbsolutePositionUsingAbsoluteEncoderAssumingTheRelativeEncoderIsntExtremelyFarOff() }, {})

        builder.addDoubleProperty("Turret Motor Supply Current", { pivotMotor.supplyCurrent.valueAsDouble }, {})

        builder.addDoubleProperty("Turret Motor Supply Voltage", { pivotMotor.supplyVoltage.valueAsDouble }, {})
        builder.addDoubleProperty("Sotm translational", { Constants.Turret.SOTM_TRANSLATIONAL_SCALAR }, { Constants.Turret.SOTM_TRANSLATIONAL_SCALAR = it })
        builder.addDoubleProperty("Sotm rotational", { Constants.Turret.SOTM_ROTATIONAL_SCALAR }, { Constants.Turret.SOTM_ROTATIONAL_SCALAR = it })
        builder.addBooleanProperty("Use sotm", { Constants.Turret.USE_SOTM }, { Constants.Turret.USE_SOTM = it })
    }
}