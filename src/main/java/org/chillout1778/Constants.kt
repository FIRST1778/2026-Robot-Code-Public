package org.chillout1778

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import java.lang.Math.toRadians
import kotlin.math.PI

object Constants {

    var TUNING_MODE = false
    var TUNING_PASSING = false

    data class PIDFCoefficients(
        var kP: Double = 0.0,
        var kI: Double = 0.0,
        var kD: Double = 0.0,
        var kV: Double = 0.0,
        var kS: Double = 0.0
    ) {
        fun createPid(): PIDController {
            return PIDController(kP, kI, kD)
        }
    }

    object CanIds {
        // INTAKE
        const val INTAKE_EXTENSION_MAIN_MOTOR = 15
        const val INTAKE_EXTENSION_FOLLOWER_MOTOR = 16
        const val INTAKE_ROLLER_MOTOR = 30

        // INDEXER
        const val SPINDEXER_MOTOR = 17
        const val BALL_TUNNEL_MOTOR = 20

        // TURRET
        const val TURRET_PIVOT_MOTOR = 25

        const val TURRET_CANCODER_1 = 27
        const val TURRET_CANCODER_2 = 26

        // SHOOTER
        const val SHOOTER_HOOD_MOTOR = 22
        const val SHOOTER_FLYWHEEL_1_MOTOR = 23
        const val SHOOTER_FLYWHEEL_2_MOTOR = 24

        // MISCELLANEOUS
        const val CANDLE = 40
    }

    object CurrentLimits {
        const val SHOOTER_FLYWHEEL = 20.0
        const val SHOOTER_HOOD = 10.0
        const val TURRET = 15.0
        const val BALL_TUNNEL = 50.0
        const val SPINDEXER = 20.0
        const val INTAKE_EXTENSION = 10.0
        const val INTAKE_ROLLERS = 20.0
        const val DRIVE = 35.0
        const val TURN = 20.0
    }

    object Field {

        // Zones when the shooter hood will retract
        val TRENCH_RETRACT_ZONES = listOf<Util.Zone2d>(
            // RED TRENCH NEAR
            Util.Zone2d(
                Translation2d(0.0, 0.0),
                Translation2d(0.0, 0.0)
            ),
            //RED TRENCH FAR
            Util.Zone2d(
                Translation2d(0.0, 0.0),
                Translation2d(0.0, 0.0)
            ),
            // BLUE TRENCH NEAR
            Util.Zone2d(
                Translation2d(0.0, 0.0),
                Translation2d(0.0, 0.0)
            ),
            // BLUE TRENCH FAR
            Util.Zone2d(
                Translation2d(0.0, 0.0),
                Translation2d(0.0, 0.0)
            ),
        )


        val BLUE_HUB = Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84))

        val RED_HUB = Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84))

        val RED_PASS_X = 12.0
        val BLUE_PASS_X = 3.5

        val RED_X_PASS_THRESHOLD = 11.56
        val BLUE_X_PASS_THRESHOLD = 4.98

        val HUB_AVOIDANCE_RANGE = (3.437 - 0.5)..(4.631 + 0.5)
    }

    //region Subsystems
    object Intake {
        const val MAX_EXTENSION_DISTANCE = -18.8 // Not sure which units this is in, it might just be in motor rotations
        const val SHOOT_SHAKE_POS = -7.0

        val EXTENSION_PID_COEFFICIENTS = PIDFCoefficients(
            0.6,
            0.0,
            0.0,
            0.0
        )

        val EXTENSION_CONFIG = TalonFXConfiguration().apply {
            CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.INTAKE_EXTENSION
            CurrentLimits.SupplyCurrentLimitEnable = true
            MotorOutput.NeutralMode = NeutralModeValue.Brake
        }

        val ROLLER_CONFIG = TalonFXConfiguration().apply {
            CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.INTAKE_ROLLERS
        }
    }

    object Indexer {
        val SPINDEXER_CONFIG = TalonFXConfiguration().apply {
            CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.SPINDEXER
        }

        val BALL_TUNNEL_CONFIG = TalonFXConfiguration().apply {
            CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.BALL_TUNNEL
        }

        val SPINDEXER_PID_COEFFICIENTS = PIDFCoefficients(
            0.001,
            0.0,
            0.0,
            0.000035,
            0.36
        )

        val BALL_TUNNEL_PID_COEFFICIENTS = PIDFCoefficients(
            0.001,
            0.0,
            0.0,
            0.00003,
            0.35
        )
    }

    object Turret {
        // The ratio between the encoders and the turret
        val EXPECTED_ENCODER_TURRET_RATIO = 135.0 / 24.0

        val TURRET_RATIO_SCALAR = 1.01069

        val ENCODER_TURRET_RATIO = EXPECTED_ENCODER_TURRET_RATIO * TURRET_RATIO_SCALAR

        // Maximum velocity to use absolute encoder
        val MAXIMUM_ABSOLUTE_ENCODER_VELOCITY = 0.01

        val DYNAMIC_OFFSET_CHANGE_SPEED = 0.015

        // Turret max rotational limit, in radians
        val TURRET_OPERATING_RANGE = toRadians(-190.0)..toRadians(190.0)

        val TURRET_PIVOT_CONFIG = TalonFXConfiguration()

        init {
            TURRET_PIVOT_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive
            TURRET_PIVOT_CONFIG.CurrentLimits.SupplyCurrentLimit = CurrentLimits.TURRET
        }

        val TURRET_PID_COEFFICIENTS = PIDFCoefficients(
            10.0,
            0.0,
            0.25,
            0.0,
            0.0
        )

        val SMALL_ENCODER_ZERO = -0.1711 // NOTE: We did not use our second encoder during competition despite our initial plans
        val BIG_ENCODER_ZERO = -0.210
        //endregion

        val MOTOR_TO_TURRET_RATIO = (46.0 / 11.0) * (135.0 / 24.0)

        var SOTM_TRANSLATIONAL_SCALAR = 1.01
        var SOTM_ROTATIONAL_SCALAR = 0.1
        var USE_SOTM = true
    }

    object Shooter {
        //region Flywheel
        val SHOOTER_FLYWHEEL_CONFIG = TalonFXConfiguration().apply {
            CurrentLimits.StatorCurrentLimit = Constants.CurrentLimits.SHOOTER_FLYWHEEL
        }
        val SHOOTER_FLYWHEEL_IDLE_RPM_MAX = 2400.0

        val SHOOTER_FLYWHEEL_PID_COEFFICIENTS = PIDFCoefficients(
            0.0001,
            0.0,
            0.0,
            0.0019,
            0.17
        )

        var ALPHA: Double = 0.2 // For first order EMA
        //endregion

        //region Hood
        val SHOOTER_HOOD_RANGE = toRadians(0.0)..toRadians(26.0)
        val SHOOTER_HOOD_REDUCTION = 27.0

        // Max angle when traversing the trench
        val HOOD_TRENCH_ANGLE = toRadians(5.0)

        val SHOOTER_HOOD_CONFIG = TalonFXConfiguration().apply {
            CurrentLimits.SupplyCurrentLimit = Constants.CurrentLimits.SHOOTER_HOOD
        }

        val SHOOTER_HOOD_PID_COEFFICIENTS = PIDFCoefficients(
            1.8,
            0.0,
            0.05,
            0.025,
            0.0
        )

        var SHOOTER_HOOD_KG = 0.3
        //endregion

        //region LUTs
        // NOTE: Distances are measured from center of rotation of the turret
        private var distancesToAngles: Array<Pair<Double, Double>> = arrayOf(
            // meters to degrees
            1.305 to 0.0,
            1.660 to 0.5,
            1.994 to 3.0,
            2.230 to 4.5,
            2.595 to 5.5,
            2.918 to 7.5,
            3.156 to 8.5,
            3.385 to 11.0,
            3.670 to 11.5,
            4.004 to 12.0,
            4.537 to 12.6,
            4.794 to 14.0,
            5.209 to 14.5,
            5.533 to 14.5,
            6.483 to 25.0,
            9.54 to 26.0
        )
        private var distancesToSpeeds: Array<Pair<Double, Double>> = arrayOf(
            // meters to RPM
            1.305 to 1800.0,
            1.660 to 1900.0,
            1.994 to 1980.0,
            2.230 to 1990.0,
            2.595 to 2000.0,
            2.918 to 2020.0,
            3.156 to 2070.0,
            3.385 to 2080.0,
            3.670 to 2120.0,
            4.004 to 2170.0,
            4.537 to 2250.0,
            4.794 to 2300.0,
            5.209 to 2400.0,
            5.533 to 2420.0,
            6.483 to 2500.0,
            9.54 to 3600.0
        )


        val angleLUT = InterpolatingDoubleTreeMap().apply {
            for((distance, angle) in distancesToAngles) {
                put(distance, toRadians(angle))
            }
        }

        val speedLUT = InterpolatingDoubleTreeMap().apply {
            for((distance, speed) in distancesToSpeeds) {
                put(distance, speed)
            }
        }
        //endregion
    }

    object Swerve {
        val MAX_SPEED = 3.0 // m/s
        val MAX_ANGULAR_RATE = 0.75 * 2 * PI // rad/s

        val DRIVE_KP = 5.0  // Made for autos
        val DRIVE_KI = 0.0
        val DRIVE_KD = 0.0

        val HEADING_KP = 5.0  // Made for autos
        val HEADING_KI = 0.0
        val HEADING_KD = 0.0
    }

    object Vision {
        // We did not end up using limelight...
        val SHOOTER_LL_NAME: String = "limelight"
        val SWERVE_TO_TURRET_TRANSFORM = Transform3d(Translation3d(-0.1363, 0.0877, 0.3302), Rotation3d())
        val TURRET_TO_LL_TRANSLATION = Translation3d(-0.850, .130115, .181)

        val LEFT_FACING_CAMERA_TRANSFORM = Transform3d(Translation3d(-0.259, 0.323,0.364), Rotation3d(0.0,toRadians(-19.853),toRadians(90.0)))
        val BACK_FACING_CAMERA_TRANSFORM = Transform3d(Translation3d(-0.323, 0.259,0.364), Rotation3d(0.0,toRadians(-19.853),toRadians(180.0)))
        val HOPPER_CAMERA_TRANSFORM = Transform3d(Translation3d(-0.323, -0.353,0.364), Rotation3d(0.0,toRadians(-19.853),toRadians(-135.0)))
        val HOPPER_CAMERA_NAME = "hopper-camera"
        val BALL_TUNNEL_BACK_NAME = "ball-tunnel-back"
        val BALL_TUNNEL_LEFT_NAME = "ball-tunnel-side"
    }

    object Lights {
        val CANDLE_LED_ZONE = 0..7
        val SHOOTER_INNER_EDGE_ZONE = 8..16
        val SHOOTER_OUTER_EDGE_ZONE = 17..25

        val ROBOT_LED_ZONE = CANDLE_LED_ZONE.first..SHOOTER_OUTER_EDGE_ZONE.last

        val LED_BRIGHTNESS = 0.7

        // TIMESTAMPS FOR THE COUNTDOWN TO THE START OF EACH PERIOD
        val SHIFT_1_COUNTDOWN_TIMESTAMP = 5.0
        val SHIFT_2_COUNTDOWN_TIMESTAMP = 30.0
        val SHIFT_3_COUNTDOWN_TIMESTAMP = 55.0
        val SHIFT_4_COUNTDOWN_TIMESTAMP = 80.0
        val ENDGAME_COUNTDOWN_TIMESTAMP = 105.0
    }
    //endregion
}