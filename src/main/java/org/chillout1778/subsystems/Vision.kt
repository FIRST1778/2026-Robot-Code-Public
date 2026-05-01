package org.chillout1778.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructPublisher
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.LimelightHelpers
import org.chillout1778.Constants
import org.chillout1778.Robot
import org.chillout1778.Robot.isRedAlliance
import org.chillout1778.Util.getDistanceToTargetTranslation
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.estimation.TargetModel
import org.photonvision.targeting.PhotonPipelineResult
import kotlin.jvm.optionals.getOrNull

object Vision : SubsystemBase() {
    
    class Camera(initialName: String, robotToCamera: Transform3d) : PhotonCamera(initialName) {
        val poseEstimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        ).apply {
            var tagModel = TargetModel.kAprilTag36h11
            setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY)
        }
    }

    private val cameras = arrayOf(
        Camera(Constants.Vision.HOPPER_CAMERA_NAME, Constants.Vision.HOPPER_CAMERA_TRANSFORM),
        Camera(Constants.Vision.BALL_TUNNEL_BACK_NAME, Constants.Vision.BACK_FACING_CAMERA_TRANSFORM),
        Camera(Constants.Vision.BALL_TUNNEL_LEFT_NAME, Constants.Vision.LEFT_FACING_CAMERA_TRANSFORM),
    )

    fun removeResult(res: PhotonPipelineResult): Boolean {
        return res.getTargets().map { it.fiducialId }.any { it in listOf<Int>() }
    }

    fun periodicAddMeasurements() {
        for (camera in cameras) {
            val results = camera.getAllUnreadResults().filterNot {
                removeResult(it)
            }
            val estimatedPoses = results.mapNotNull { camera.poseEstimator.update(it).getOrNull() }
            for (pose in estimatedPoses) {
                if(/*pose.estimatedPose.translation.toTranslation2d().isInsideField()
                    &&*/ (pose.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                            (pose.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                                    && pose.targetsUsed[0].poseAmbiguity < 0.05
                                    && pose.targetsUsed[0].area > 0.05
                                    )
                            )
                ){
                    Swerve.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, VecBuilder.fill(0.5, 0.5, 1.0))
                }
            }
        }
    }

    var turretPose = Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))

    val estimatedPosePublisher : StructPublisher<Pose2d> = NetworkTableInstance.getDefault().getStructTopic("/Field/RobotPose", Pose2d.struct).publish();

    val cameraPosePublisher : StructPublisher<Pose3d> = NetworkTableInstance.getDefault().getStructTopic("/Field/CameraPose", Pose3d.struct).publish();

    override fun periodic() {
        // Update Robot pose
        LimelightHelpers.SetRobotOrientation(
            Constants.Vision.SHOOTER_LL_NAME,
            yaw = Robot.swervePose.rotation.degrees,
            yawRate = 0.0,
            pitch = 0.0,
            pitchRate = 0.0,
            roll = 0.0,
            rollRate = 0.0,
        )


        // Assume a constant shooter for now
        turretPose = Pose3d(Robot.swervePose).plus(Constants.Vision.LEFT_FACING_CAMERA_TRANSFORM)

        periodicAddMeasurements()
        estimatedPosePublisher.set(Robot.swervePose)
        cameraPosePublisher.set(turretPose)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.addDoubleProperty("Distance to target", { getDistanceToTargetTranslation(
            if (Constants.TUNING_PASSING)
                Translation2d(if (isRedAlliance) Constants.Field.RED_PASS_X else Constants.Field.BLUE_PASS_X,
                    Robot.swervePose.y)
            else if (isRedAlliance) Constants.Field.RED_HUB else Constants.Field.BLUE_HUB
        ).first
        }, {})
        builder.addDoubleProperty("Angle to target", { getDistanceToTargetTranslation(
            if (Constants.TUNING_PASSING)
                Translation2d(if (isRedAlliance) Constants.Field.RED_PASS_X else Constants.Field.BLUE_PASS_X,
                    Robot.swervePose.y)
            else if (isRedAlliance) Constants.Field.RED_HUB else Constants.Field.BLUE_HUB
        ).second }, {})
    }
}