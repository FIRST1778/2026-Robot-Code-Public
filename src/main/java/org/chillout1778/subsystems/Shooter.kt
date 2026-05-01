package org.chillout1778.subsystems

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.chillout1778.Constants
import org.chillout1778.Controls
import org.chillout1778.Robot
import org.chillout1778.Util
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

object Shooter : SubsystemBase() {

    val hoodMotor = TalonFX(Constants.CanIds.SHOOTER_HOOD_MOTOR)
    val flywheel1Motor = TalonFX(Constants.CanIds.SHOOTER_FLYWHEEL_1_MOTOR)
    val flywheel2Motor = TalonFX(Constants.CanIds.SHOOTER_FLYWHEEL_2_MOTOR)

    private var hoodSetpoint = 0.0 // radians
    private var hoodSetpointForTesting = 0.0

    private var flywheelVelocityForTestShooting = 0.0 // rpm
    var flywheelVelocity = 0.0 // rotations/min

    var flywheelPid = Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.createPid()
    val hoodPid = Constants.Shooter.SHOOTER_HOOD_PID_COEFFICIENTS.createPid()

    var atSetpoint = false

    init {
        hoodMotor.configurator.apply { Constants.Shooter.SHOOTER_HOOD_CONFIG }
        flywheel1Motor.configurator.apply { Constants.Shooter.SHOOTER_FLYWHEEL_CONFIG }
        flywheel2Motor.configurator.apply { Constants.Shooter.SHOOTER_FLYWHEEL_CONFIG }
        flywheel2Motor.apply { setControl(Follower(flywheel1Motor.deviceID, MotorAlignmentValue.Aligned)) }

        hoodMotor.setPosition(Constants.Shooter.SHOOTER_HOOD_RANGE.start)
        hoodMotor.setNeutralMode(NeutralModeValue.Brake)

        hoodMotor.optimizeBusUtilization()
        flywheel1Motor.optimizeBusUtilization()
        flywheel2Motor.optimizeBusUtilization()

        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            hoodMotor.position, hoodMotor.velocity, hoodMotor.motorVoltage,
            flywheel1Motor.position, flywheel1Motor.velocity, flywheel1Motor.motorVoltage,
            flywheel2Motor.position, flywheel2Motor.velocity, flywheel2Motor.motorVoltage)
        BaseStatusSignal.setUpdateFrequencyForAll(10.0,
            hoodMotor.supplyCurrent, hoodMotor.supplyVoltage,
            flywheel1Motor.supplyCurrent, flywheel1Motor.supplyVoltage,
            flywheel2Motor.supplyCurrent, flywheel2Motor.supplyVoltage)

    }

    override fun periodic() {

        if (Constants.TUNING_MODE) {
            flywheelVelocity = flywheelVelocityForTestShooting
            hoodSetpoint = hoodSetpointForTesting
        }

        hoodPid.setpoint = (hoodSetpoint.coerceIn(Constants.Shooter.SHOOTER_HOOD_RANGE) * Constants.Shooter.SHOOTER_HOOD_REDUCTION) / (PI * 2)

        flywheelPid.setpoint = flywheelVelocity // firstOrderEma(Constants.Shooter.ALPHA, flywheelVelocity)

        flywheel1Motor.setVoltage((if (Controls.controls.wantFlywheelReverse) -1 else 1) * (
            flywheelPid.calculate(flywheel1Motor.velocity.valueAsDouble * 60)
            + (flywheelPid.setpoint) * Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kV
            + sign(flywheelPid.setpoint) * Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kS
                )
        )

        hoodMotor.setVoltage(
            hoodPid.calculate(hoodMotor.position.valueAsDouble)
            + sign(hoodPid.setpoint - hoodMotor.position.valueAsDouble) * Constants.Shooter.SHOOTER_HOOD_PID_COEFFICIENTS.kS
            + Constants.Shooter.SHOOTER_HOOD_KG
        )

        atSetpoint = (abs(flywheelPid.setpoint - (flywheel1Motor.velocity.valueAsDouble * 60.0)) < 40.0) && (abs(hoodPid.setpoint - hoodMotor.position.valueAsDouble) < 0.4)
    }

    // Target the set projectile exit angle, in radians
    fun moveHoodTo(angle: Double) {

        // By default, allow the full range of motion to the hood
        var allowedRange = Constants.Shooter.SHOOTER_HOOD_RANGE

        // Check if the robot is in zones where the hood should be capped,
        // set a new cap if it is
        Constants.Field.TRENCH_RETRACT_ZONES.forEach {
            if (Util.poseInZone(Robot.swervePose, it))
                allowedRange = 0.0..Constants.Shooter.HOOD_TRENCH_ANGLE
        }

        // rotate to the calculated number, after clamping down into allowed zones
        hoodSetpoint = angle.coerceIn(allowedRange)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addBooleanProperty("Hood error tolerance", { abs(hoodPid.setpoint - hoodMotor.position.valueAsDouble) < 0.002 }, { })

        builder.addDoubleProperty("Hood setpoint degrees", { (hoodSetpointForTesting / (2 * PI)) * 360 }, { hoodSetpointForTesting = (it / 360) * 2 * PI })
        builder.addDoubleProperty("Shooter setpoint rpm", { flywheelVelocity }, { flywheelVelocity = it })
        builder.addDoubleProperty("Hood actual degrees", { hoodMotor.position.valueAsDouble / Constants.Shooter.SHOOTER_HOOD_REDUCTION * 360 }, { })

        builder.addDoubleProperty("Flywheel Shoot Velocity", { flywheelVelocityForTestShooting }, { flywheelVelocityForTestShooting = it })

        builder.addDoubleProperty("Flywheel actual velocity", { flywheel1Motor.velocity.valueAsDouble * 60 }, {})
        builder.addDoubleProperty("Setpoint", { flywheelPid.setpoint }, {})

        builder.addDoubleProperty("hood kS", { Constants.Shooter.SHOOTER_HOOD_PID_COEFFICIENTS.kS }, { Constants.Shooter.SHOOTER_HOOD_PID_COEFFICIENTS.kS = it })
        builder.addDoubleProperty("hood kG", { Constants.Shooter.SHOOTER_HOOD_KG }, { Constants.Shooter.SHOOTER_HOOD_KG = it })

        builder.addDoubleProperty("hood kP", { Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kP }, {
            Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kP = it
            flywheelPid.p = Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kP
        })

        builder.addDoubleProperty("hood kD", { Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kD }, {
            Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kD = it
            flywheelPid.d = Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kD
        })
        builder.addDoubleProperty("flywheel kV", { Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kV }, {
            Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kV = it
        })

        builder.addDoubleProperty("flywheel kS", { Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kS }, {
            Constants.Shooter.SHOOTER_FLYWHEEL_PID_COEFFICIENTS.kS = it
        })
        builder.addDoubleProperty("flywheel alpha", { Constants.Shooter.ALPHA }, { Constants.Shooter.ALPHA = it })


        builder.addDoubleProperty("hood kV", { Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kV }, {
            Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kV = it
            hoodMotor.configurator.apply(Constants.Shooter.SHOOTER_HOOD_CONFIG)
        })

        builder.addDoubleProperty("hood kStatic", { Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kS }, {
            Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kS = it
            hoodMotor.configurator.apply(Constants.Shooter.SHOOTER_HOOD_CONFIG)
        })

        builder.addDoubleProperty("hood kP", { Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kP }, {
            Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kP = it
            hoodMotor.configurator.apply(Constants.Shooter.SHOOTER_HOOD_CONFIG)
        })

        builder.addDoubleProperty("hood kD", { Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kD }, {
            Constants.Shooter.SHOOTER_HOOD_CONFIG.Slot0.kD = it
            hoodMotor.configurator.apply(Constants.Shooter.SHOOTER_HOOD_CONFIG)
        })

        builder.addDoubleProperty("Shooter Hood Supply Current", { hoodMotor.supplyCurrent.valueAsDouble}, {})
        builder.addDoubleProperty("Shooter Flywheel 1 Supply Current", { flywheel1Motor.supplyCurrent.valueAsDouble}, {})
        builder.addDoubleProperty("Shooter Flywheel 2 Supply Current", { flywheel2Motor.supplyCurrent.valueAsDouble}, {})

        builder.addDoubleProperty("Shooter Hood Supply Voltage", { hoodMotor.supplyVoltage.valueAsDouble}, {})
        builder.addDoubleProperty("Shooter Flywheel 1 Supply Voltage", { flywheel1Motor.supplyVoltage.valueAsDouble}, {})
        builder.addDoubleProperty("Shooter Flywheel 2 Supply Voltage", { flywheel2Motor.supplyVoltage.valueAsDouble}, {})
    }

    var lastReference: Double = 0.0
    fun firstOrderEma(alpha: Double, goal: Double): Double {
        lastReference = goal * alpha + lastReference * (1 - alpha)
        return lastReference
    }
}