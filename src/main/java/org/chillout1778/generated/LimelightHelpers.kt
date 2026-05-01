//LimelightHelpers v1.14 (REQUIRES LLOS 2026.0 OR LATER)
package frc.robot

import com.fasterxml.jackson.annotation.JsonFormat
import com.fasterxml.jackson.annotation.JsonProperty
import com.fasterxml.jackson.core.JsonProcessingException
import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.util.Units
import edu.wpi.first.net.PortForwarder
import edu.wpi.first.networktables.DoubleArrayEntry
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.NetworkTableInstance
import java.util.concurrent.ConcurrentHashMap
import kotlin.math.min

/**
 * LimelightHelpers provides static methods and classes for interfacing with Limelight vision cameras in FRC.
 * This library supports all Limelight features including AprilTag tracking, Neural Networks, and standard color/retroreflective tracking.
 */
object LimelightHelpers {
    private val doubleArrayEntries: MutableMap<String, DoubleArrayEntry> = ConcurrentHashMap()

    private var mapper: ObjectMapper? = null

    /**
     * Print JSON Parse time to the console in milliseconds
     */
    var profileJSON: Boolean = false

    fun sanitizeName(name: String?): String {
        if ("" == name || name == null) {
            return "limelight"
        }
        return name
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose3d object.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose3d object representing the pose, or empty Pose3d if invalid data
     */
    fun toPose3D(inData: DoubleArray): Pose3d {
        if (inData.size < 6) {
            //System.err.println("Bad LL 3D Pose Data!");
            return Pose3d()
        }
        return Pose3d(
            Translation3d(inData[0], inData[1], inData[2]),
            Rotation3d(
                Units.degreesToRadians(inData[3]), Units.degreesToRadians(
                    inData[4]
                ),
                Units.degreesToRadians(inData[5])
            )
        )
    }

    /**
     * Takes a 6-length array of pose data and converts it to a Pose2d object.
     * Uses only x, y, and yaw components, ignoring z, roll, and pitch.
     * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
     * @param inData Array containing pose data [x, y, z, roll, pitch, yaw]
     * @return Pose2d object representing the pose, or empty Pose2d if invalid data
     */
    fun toPose2D(inData: DoubleArray): Pose2d {
        if (inData.size < 6) {
            //System.err.println("Bad LL 2D Pose Data!");
            return Pose2d()
        }
        val tran2d = Translation2d(inData[0], inData[1])
        val r2d = Rotation2d(Units.degreesToRadians(inData[5]))
        return Pose2d(tran2d, r2d)
    }

    /**
     * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     *
     * @param pose The Pose3d object to convert
     * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
     */
    fun pose3dToArray(pose: Pose3d): DoubleArray {
        val result = DoubleArray(6)
        result[0] = pose.translation.x
        result[1] = pose.translation.y
        result[2] = pose.translation.z
        result[3] = Units.radiansToDegrees(pose.rotation.x)
        result[4] = Units.radiansToDegrees(pose.rotation.y)
        result[5] = Units.radiansToDegrees(pose.rotation.z)
        return result
    }

    /**
     * Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
     * Translation components are in meters, rotation components are in degrees.
     * Note: z, roll, and pitch will be 0 since Pose2d only contains x, y, and yaw.
     *
     * @param pose The Pose2d object to convert
     * @return A 6-element array containing [x, y, 0, 0, 0, yaw]
     */
    fun pose2dToArray(pose: Pose2d): DoubleArray {
        val result = DoubleArray(6)
        result[0] = pose.translation.x
        result[1] = pose.translation.y
        result[2] = 0.0
        result[3] = Units.radiansToDegrees(0.0)
        result[4] = Units.radiansToDegrees(0.0)
        result[5] = Units.radiansToDegrees(pose.rotation.radians)
        return result
    }

    private fun extractArrayEntry(inData: DoubleArray, position: Int): Double {
        if (inData.size < position + 1) {
            return 0.0
        }
        return inData[position]
    }

    private fun getBotPoseEstimate(limelightName: String, entryName: String, isMegaTag2: Boolean): PoseEstimate {
        val poseEntry = getLimelightDoubleArrayEntry(limelightName, entryName)

        val tsValue = poseEntry.atomic
        val poseArray = tsValue.value
        val timestamp = tsValue.timestamp

        if (poseArray.size == 0) {
            // Handle the case where no data is available
            return PoseEstimate()
        }

        val pose = toPose2D(poseArray)
        val latency = extractArrayEntry(poseArray, 6)
        val tagCount = extractArrayEntry(poseArray, 7).toInt()
        val tagSpan = extractArrayEntry(poseArray, 8)
        val tagDist = extractArrayEntry(poseArray, 9)
        val tagArea = extractArrayEntry(poseArray, 10)


        // Convert server timestamp from microseconds to seconds and adjust for latency
        val adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0)

        val valsPerFiducial = 7
        val expectedTotalVals = 11 + valsPerFiducial * tagCount
        val rawFiducials: Array<RawFiducial?>

        if (poseArray.size != expectedTotalVals) {
            // Array size mismatch - return empty array instead of null-filled array
            rawFiducials = arrayOfNulls(0)
        } else {
            rawFiducials = arrayOfNulls(tagCount)
            for (i in 0 until tagCount) {
                val baseIndex = 11 + (i * valsPerFiducial)
                val id = poseArray[baseIndex].toInt()
                val txnc = poseArray[baseIndex + 1]
                val tync = poseArray[baseIndex + 2]
                val ta = poseArray[baseIndex + 3]
                val distToCamera = poseArray[baseIndex + 4]
                val distToRobot = poseArray[baseIndex + 5]
                val ambiguity = poseArray[baseIndex + 6]
                rawFiducials[i] = RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity)
            }
        }

        return PoseEstimate(
            pose,
            adjustedTimestamp,
            latency,
            tagCount,
            tagSpan,
            tagDist,
            tagArea,
            rawFiducials,
            isMegaTag2
        )
    }

    /**
     * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of RawFiducial objects containing detection details
     */
    fun getRawFiducials(limelightName: String?): Array<RawFiducial?> {
        val entry = getLimelightNTTableEntry(limelightName, "rawfiducials")
        val rawFiducialArray = entry.getDoubleArray(DoubleArray(0))
        val valsPerEntry = 7
        if (rawFiducialArray.size % valsPerEntry != 0) {
            return arrayOfNulls(0)
        }

        val numFiducials = rawFiducialArray.size / valsPerEntry
        val rawFiducials = arrayOfNulls<RawFiducial>(numFiducials)

        for (i in 0 until numFiducials) {
            val baseIndex = i * valsPerEntry
            val id = extractArrayEntry(rawFiducialArray, baseIndex).toInt()
            val txnc = extractArrayEntry(rawFiducialArray, baseIndex + 1)
            val tync = extractArrayEntry(rawFiducialArray, baseIndex + 2)
            val ta = extractArrayEntry(rawFiducialArray, baseIndex + 3)
            val distToCamera = extractArrayEntry(rawFiducialArray, baseIndex + 4)
            val distToRobot = extractArrayEntry(rawFiducialArray, baseIndex + 5)
            val ambiguity = extractArrayEntry(rawFiducialArray, baseIndex + 6)

            rawFiducials[i] = RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity)
        }

        return rawFiducials
    }

    /**
     * Gets the latest raw neural detector results from NetworkTables
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of RawDetection objects containing detection details
     */
    fun getRawDetections(limelightName: String?): Array<RawDetection?> {
        val entry = getLimelightNTTableEntry(limelightName, "rawdetections")
        val rawDetectionArray = entry.getDoubleArray(DoubleArray(0))
        val valsPerEntry = 12
        if (rawDetectionArray.size % valsPerEntry != 0) {
            return arrayOfNulls(0)
        }

        val numDetections = rawDetectionArray.size / valsPerEntry
        val rawDetections = arrayOfNulls<RawDetection>(numDetections)

        for (i in 0 until numDetections) {
            val baseIndex = i * valsPerEntry // Starting index for this detection's data
            val classId = extractArrayEntry(rawDetectionArray, baseIndex).toInt()
            val txnc = extractArrayEntry(rawDetectionArray, baseIndex + 1)
            val tync = extractArrayEntry(rawDetectionArray, baseIndex + 2)
            val ta = extractArrayEntry(rawDetectionArray, baseIndex + 3)
            val corner0_X = extractArrayEntry(rawDetectionArray, baseIndex + 4)
            val corner0_Y = extractArrayEntry(rawDetectionArray, baseIndex + 5)
            val corner1_X = extractArrayEntry(rawDetectionArray, baseIndex + 6)
            val corner1_Y = extractArrayEntry(rawDetectionArray, baseIndex + 7)
            val corner2_X = extractArrayEntry(rawDetectionArray, baseIndex + 8)
            val corner2_Y = extractArrayEntry(rawDetectionArray, baseIndex + 9)
            val corner3_X = extractArrayEntry(rawDetectionArray, baseIndex + 10)
            val corner3_Y = extractArrayEntry(rawDetectionArray, baseIndex + 11)

            rawDetections[i] = RawDetection(
                classId,
                txnc,
                tync,
                ta,
                corner0_X,
                corner0_Y,
                corner1_X,
                corner1_Y,
                corner2_X,
                corner2_Y,
                corner3_X,
                corner3_Y
            )
        }

        return rawDetections
    }

    /**
     * Gets the raw target contours from NetworkTables.
     * Returns ungrouped contours in normalized screen space (-1 to 1).
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of RawTarget objects containing up to 3 contours
     */
    fun getRawTargets(limelightName: String?): Array<RawTarget?> {
        val entry = getLimelightNTTableEntry(limelightName, "rawtargets")
        val rawTargetArray = entry.getDoubleArray(DoubleArray(0))
        val valsPerEntry = 3
        if (rawTargetArray.size % valsPerEntry != 0) {
            return arrayOfNulls(0)
        }

        val numTargets = rawTargetArray.size / valsPerEntry
        val rawTargets = arrayOfNulls<RawTarget>(numTargets)

        for (i in 0 until numTargets) {
            val baseIndex = i * valsPerEntry
            val txnc = extractArrayEntry(rawTargetArray, baseIndex)
            val tync = extractArrayEntry(rawTargetArray, baseIndex + 1)
            val ta = extractArrayEntry(rawTargetArray, baseIndex + 2)

            rawTargets[i] = RawTarget(txnc, tync, ta)
        }

        return rawTargets
    }

    /**
     * Gets the corner coordinates of detected targets from NetworkTables.
     * Requires "send contours" to be enabled in the Limelight Output tab.
     *
     * @param limelightName Name/identifier of the Limelight
     * @return Array of doubles containing corner coordinates [x0, y0, x1, y1, ...]
     */
    fun getCornerCoordinates(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "tcornxy")
    }

    /**
     * Prints detailed information about a PoseEstimate to standard output.
     * Includes timestamp, latency, tag count, tag span, average tag distance,
     * average tag area, and detailed information about each detected fiducial.
     *
     * @param pose The PoseEstimate object to print. If null, prints "No PoseEstimate available."
     */
    fun printPoseEstimate(pose: PoseEstimate?) {
        if (pose == null) {
            println("No PoseEstimate available.")
            return
        }

        System.out.printf("Pose Estimate Information:%n")
        System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds)
        System.out.printf("Latency: %.3f ms%n", pose.latency)
        System.out.printf("Tag Count: %d%n", pose.tagCount)
        System.out.printf("Tag Span: %.2f meters%n", pose.tagSpan)
        System.out.printf("Average Tag Distance: %.2f meters%n", pose.avgTagDist)
        System.out.printf("Average Tag Area: %.2f%% of image%n", pose.avgTagArea)
        System.out.printf("Is MegaTag2: %b%n", pose.isMegaTag2)
        println()

        if (pose.rawFiducials == null || pose.rawFiducials!!.size == 0) {
            println("No RawFiducials data available.")
            return
        }

        println("Raw Fiducials Details:")
        for (i in pose.rawFiducials!!.indices) {
            val fiducial = pose.rawFiducials!![i]
            System.out.printf(" Fiducial #%d:%n", i + 1)
            System.out.printf("  ID: %d%n", fiducial!!.id)
            System.out.printf("  TXNC: %.2f%n", fiducial.txnc)
            System.out.printf("  TYNC: %.2f%n", fiducial.tync)
            System.out.printf("  TA: %.2f%n", fiducial.ta)
            System.out.printf("  Distance to Camera: %.2f meters%n", fiducial.distToCamera)
            System.out.printf("  Distance to Robot: %.2f meters%n", fiducial.distToRobot)
            System.out.printf("  Ambiguity: %.2f%n", fiducial.ambiguity)
            println()
        }
    }

    fun validPoseEstimate(pose: PoseEstimate?): Boolean {
        return pose?.rawFiducials != null && pose.rawFiducials!!.size != 0
    }

    fun getLimelightNTTable(tableName: String?): NetworkTable {
        return NetworkTableInstance.getDefault().getTable(sanitizeName(tableName))
    }

    fun Flush() {
        NetworkTableInstance.getDefault().flush()
    }

    fun getLimelightNTTableEntry(tableName: String?, entryName: String?): NetworkTableEntry {
        return getLimelightNTTable(tableName).getEntry(entryName)
    }

    fun getLimelightDoubleArrayEntry(tableName: String, entryName: String): DoubleArrayEntry {
        val key = "$tableName/$entryName"
        return doubleArrayEntries.computeIfAbsent(key) { k: String? ->
            val table = getLimelightNTTable(tableName)
            table.getDoubleArrayTopic(entryName).getEntry(DoubleArray(0))
        }
    }

    fun getLimelightNTDouble(tableName: String?, entryName: String?): Double {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0)
    }

    fun setLimelightNTDouble(tableName: String?, entryName: String?, `val`: Double) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(`val`)
    }

    fun setLimelightNTDoubleArray(tableName: String?, entryName: String?, `val`: DoubleArray?) {
        getLimelightNTTableEntry(tableName, entryName).setDoubleArray(`val`)
    }

    fun getLimelightNTDoubleArray(tableName: String?, entryName: String?): DoubleArray {
        return getLimelightNTTableEntry(tableName, entryName).getDoubleArray(DoubleArray(0))
    }


    fun getLimelightNTString(tableName: String?, entryName: String?): String {
        return getLimelightNTTableEntry(tableName, entryName).getString("")
    }

    fun getLimelightNTStringArray(tableName: String?, entryName: String?): Array<String> {
        return getLimelightNTTableEntry(tableName, entryName).getStringArray(arrayOfNulls(0))
    }


    /////
    /**
     * Does the Limelight have a valid target?
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return True if a valid target is present, false otherwise
     */
    fun getTV(limelightName: String?): Boolean {
        return 1.0 == getLimelightNTDouble(limelightName, "tv")
    }

    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    fun getTX(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "tx")
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    fun getTY(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "ty")
    }

    /**
     * Gets the horizontal offset from the principal pixel/point to the target in degrees.  This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Horizontal offset angle in degrees
     */
    fun getTXNC(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "txnc")
    }

    /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Vertical offset angle in degrees
     */
    fun getTYNC(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "tync")
    }

    /**
     * Gets the target area as a percentage of the image (0-100%).
     * @param limelightName Name of the Limelight camera ("" for default)
     * @return Target area percentage (0-100)
     */
    fun getTA(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "ta")
    }

    /**
     * T2D is an array that contains several targeting metrcis
     * @param limelightName Name of the Limelight camera
     * @return Array containing  [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector,
     * targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
     */
    fun getT2DArray(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "t2d")
    }

    /**
     * Gets the number of targets currently detected.
     * @param limelightName Name of the Limelight camera
     * @return Number of detected targets
     */
    fun getTargetCount(limelightName: String?): Int {
        val t2d = getT2DArray(limelightName)
        if (t2d.size == 17) {
            return t2d[1].toInt()
        }
        return 0
    }

    /**
     * Gets the classifier class index from the currently running neural classifier pipeline
     * @param limelightName Name of the Limelight camera
     * @return Class index from classifier pipeline
     */
    fun getClassifierClassIndex(limelightName: String?): Int {
        val t2d = getT2DArray(limelightName)
        if (t2d.size == 17) {
            return t2d[11].toInt()
        }
        return 0
    }

    /**
     * Gets the detector class index from the primary result of the currently running neural detector pipeline.
     * @param limelightName Name of the Limelight camera
     * @return Class index from detector pipeline
     */
    fun getDetectorClassIndex(limelightName: String?): Int {
        val t2d = getT2DArray(limelightName)
        if (t2d.size == 17) {
            return t2d[10].toInt()
        }
        return 0
    }

    /**
     * Gets the current neural classifier result class name.
     * @param limelightName Name of the Limelight camera
     * @return Class name string from classifier pipeline
     */
    fun getClassifierClass(limelightName: String?): String {
        return getLimelightNTString(limelightName, "tcclass")
    }

    /**
     * Gets the primary neural detector result class name.
     * @param limelightName Name of the Limelight camera
     * @return Class name string from detector pipeline
     */
    fun getDetectorClass(limelightName: String?): String {
        return getLimelightNTString(limelightName, "tdclass")
    }

    /**
     * Gets the pipeline's processing latency contribution.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline latency in milliseconds
     */
    fun getLatency_Pipeline(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "tl")
    }

    /**
     * Gets the capture latency.
     * @param limelightName Name of the Limelight camera
     * @return Capture latency in milliseconds
     */
    fun getLatency_Capture(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "cl")
    }

    /**
     * Gets the active pipeline index.
     * @param limelightName Name of the Limelight camera
     * @return Current pipeline index (0-9)
     */
    fun getCurrentPipelineIndex(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "getpipe")
    }

    /**
     * Gets the current pipeline type.
     * @param limelightName Name of the Limelight camera
     * @return Pipeline type string (e.g. "retro", "apriltag", etc)
     */
    fun getCurrentPipelineType(limelightName: String?): String {
        return getLimelightNTString(limelightName, "getpipetype")
    }

    /**
     * Gets the full JSON results dump.
     * @param limelightName Name of the Limelight camera
     * @return JSON string containing all current results
     */
    fun getJSONDump(limelightName: String?): String {
        return getLimelightNTString(limelightName, "json")
    }

    /**
     * Switch to getBotPose
     *
     * @param limelightName
     * @return
     */
    @Deprecated("")
    fun getBotpose(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose")
    }

    /**
     * Switch to getBotPose_wpiRed
     *
     * @param limelightName
     * @return
     */
    @Deprecated("")
    fun getBotpose_wpiRed(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired")
    }

    /**
     * Switch to getBotPose_wpiBlue
     *
     * @param limelightName
     * @return
     */
    @Deprecated("")
    fun getBotpose_wpiBlue(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue")
    }

    fun getBotPose(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose")
    }

    fun getBotPose_wpiRed(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired")
    }

    fun getBotPose_wpiBlue(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue")
    }

    fun getBotPose_TargetSpace(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetspace")
    }

    fun getCameraPose_TargetSpace(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace")
    }

    fun getTargetPose_CameraSpace(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace")
    }

    fun getTargetPose_RobotSpace(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace")
    }

    /**
     * Gets the average color under the crosshair region as a 3-element array.
     * @param limelightName Name of the Limelight camera
     * @return Array containing [Blue, Green, Red] color values (BGR order)
     */
    fun getTargetColor(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "tc")
    }

    fun getFiducialID(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "tid")
    }

    /**
     * Gets the Limelight heartbeat value. Increments once per frame, allowing you to detect if the Limelight is connected and alive.
     * @param limelightName Name of the Limelight camera
     * @return Heartbeat value that increments each frame
     */
    fun getHeartbeat(limelightName: String?): Double {
        return getLimelightNTDouble(limelightName, "hb")
    }

    fun getNeuralClassID(limelightName: String?): String {
        return getLimelightNTString(limelightName, "tclass")
    }

    fun getRawBarcodeData(limelightName: String?): Array<String> {
        return getLimelightNTStringArray(limelightName, "rawbarcodes")
    }

    /////
    /////
    fun getBotPose3d(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "botpose")
        return toPose3D(poseArray)
    }

    /**
     * (Not Recommended) Gets the robot's 3D pose in the WPILib Red Alliance Coordinate System.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation in Red Alliance field space
     */
    fun getBotPose3d_wpiRed(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpired")
        return toPose3D(poseArray)
    }

    /**
     * (Recommended) Gets the robot's 3D pose in the WPILib Blue Alliance Coordinate System.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation in Blue Alliance field space
     */
    fun getBotPose3d_wpiBlue(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "botpose_wpiblue")
        return toPose3D(poseArray)
    }

    /**
     * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the robot's position and orientation relative to the target
     */
    fun getBotPose3d_TargetSpace(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "botpose_targetspace")
        return toPose3D(poseArray)
    }

    /**
     * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the camera's position and orientation relative to the target
     */
    fun getCameraPose3d_TargetSpace(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_targetspace")
        return toPose3D(poseArray)
    }

    /**
     * Gets the target's 3D pose with respect to the camera's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the camera
     */
    fun getTargetPose3d_CameraSpace(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace")
        return toPose3D(poseArray)
    }

    /**
     * Gets the target's 3D pose with respect to the robot's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the target's position and orientation relative to the robot
     */
    fun getTargetPose3d_RobotSpace(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "targetpose_robotspace")
        return toPose3D(poseArray)
    }

    /**
     * Gets the camera's 3D pose with respect to the robot's coordinate system.
     * @param limelightName Name/identifier of the Limelight
     * @return Pose3d object representing the camera's position and orientation relative to the robot
     */
    fun getCameraPose3d_RobotSpace(limelightName: String?): Pose3d {
        val poseArray = getLimelightNTDoubleArray(limelightName, "camerapose_robotspace")
        return toPose3D(poseArray)
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     *
     * @param limelightName
     * @return
     */
    fun getBotPose2d_wpiBlue(limelightName: String?): Pose2d {
        val result = getBotPose_wpiBlue(limelightName)
        return toPose2D(result)
    }

    /**
     * Gets the MegaTag1 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     *
     * @param limelightName
     * @return
     */
    fun getBotPoseEstimate_wpiBlue(limelightName: String): PoseEstimate {
        return getBotPoseEstimate(limelightName, "botpose_wpiblue", false)
    }

    /**
     * Gets the MegaTag2 Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) in the WPILib Blue alliance coordinate system.
     * Make sure you are calling setRobotOrientation() before calling this method.
     *
     * @param limelightName
     * @return
     */
    fun getBotPoseEstimate_wpiBlue_MegaTag2(limelightName: String): PoseEstimate {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpiblue", true)
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     *
     * @param limelightName
     * @return
     */
    fun getBotPose2d_wpiRed(limelightName: String?): Pose2d {
        val result = getBotPose_wpiRed(limelightName)
        return toPose2D(result)
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
     * alliance
     * @param limelightName
     * @return
     */
    fun getBotPoseEstimate_wpiRed(limelightName: String): PoseEstimate {
        return getBotPoseEstimate(limelightName, "botpose_wpired", false)
    }

    /**
     * Gets the Pose2d and timestamp for use with WPILib pose estimator (addVisionMeasurement) when you are on the RED
     * alliance
     * @param limelightName
     * @return
     */
    fun getBotPoseEstimate_wpiRed_MegaTag2(limelightName: String): PoseEstimate {
        return getBotPoseEstimate(limelightName, "botpose_orb_wpired", true)
    }

    /**
     * Gets the Pose2d for easy use with Odometry vision pose estimator
     * (addVisionMeasurement)
     *
     * @param limelightName
     * @return
     */
    fun getBotPose2d(limelightName: String?): Pose2d {
        val result = getBotPose(limelightName)
        return toPose2D(result)
    }

    /**
     * Gets the current IMU data from NetworkTables.
     * IMU data is formatted as [robotYaw, Roll, Pitch, Yaw, gyroX, gyroY, gyroZ, accelX, accelY, accelZ].
     * Returns all zeros if data is invalid or unavailable.
     *
     * @param limelightName Name/identifier of the Limelight
     * @return IMUData object containing all current IMU data
     */
    fun getIMUData(limelightName: String?): IMUData {
        val imuData = getLimelightNTDoubleArray(limelightName, "imu")
        if (imuData == null || imuData.size < 10) {
            return IMUData() // Returns object with all zeros
        }
        return IMUData(imuData)
    }

    /////
    /////
    fun setPipelineIndex(limelightName: String?, pipelineIndex: Int) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex.toDouble())
    }


    fun setPriorityTagID(limelightName: String?, ID: Int) {
        setLimelightNTDouble(limelightName, "priorityid", ID.toDouble())
    }

    /**
     * Sets LED mode to be controlled by the current pipeline.
     * @param limelightName Name of the Limelight camera
     */
    fun setLEDMode_PipelineControl(limelightName: String?) {
        setLimelightNTDouble(limelightName, "ledMode", 0.0)
    }

    fun setLEDMode_ForceOff(limelightName: String?) {
        setLimelightNTDouble(limelightName, "ledMode", 1.0)
    }

    fun setLEDMode_ForceBlink(limelightName: String?) {
        setLimelightNTDouble(limelightName, "ledMode", 2.0)
    }

    fun setLEDMode_ForceOn(limelightName: String?) {
        setLimelightNTDouble(limelightName, "ledMode", 3.0)
    }

    /**
     * Enables standard side-by-side stream mode.
     * @param limelightName Name of the Limelight camera
     */
    fun setStreamMode_Standard(limelightName: String?) {
        setLimelightNTDouble(limelightName, "stream", 0.0)
    }

    /**
     * Enables Picture-in-Picture mode with secondary stream in the corner.
     * @param limelightName Name of the Limelight camera
     */
    fun setStreamMode_PiPMain(limelightName: String?) {
        setLimelightNTDouble(limelightName, "stream", 1.0)
    }

    /**
     * Enables Picture-in-Picture mode with primary stream in the corner.
     * @param limelightName Name of the Limelight camera
     */
    fun setStreamMode_PiPSecondary(limelightName: String?) {
        setLimelightNTDouble(limelightName, "stream", 2.0)
    }


    /**
     * Sets the crop window for the camera. The crop window in the UI must be completely open.
     * @param limelightName Name of the Limelight camera
     * @param cropXMin Minimum X value (-1 to 1)
     * @param cropXMax Maximum X value (-1 to 1)
     * @param cropYMin Minimum Y value (-1 to 1)
     * @param cropYMax Maximum Y value (-1 to 1)
     */
    fun setCropWindow(limelightName: String?, cropXMin: Double, cropXMax: Double, cropYMin: Double, cropYMax: Double) {
        val entries = DoubleArray(4)
        entries[0] = cropXMin
        entries[1] = cropXMax
        entries[2] = cropYMin
        entries[3] = cropYMax
        setLimelightNTDoubleArray(limelightName, "crop", entries)
    }

    /**
     * Sets the keystone modification for the crop window.
     * @param limelightName Name of the Limelight camera
     * @param horizontal Horizontal keystone value (-0.95 to 0.95)
     * @param vertical Vertical keystone value (-0.95 to 0.95)
     */
    fun setKeystone(limelightName: String?, horizontal: Double, vertical: Double) {
        val entries = DoubleArray(2)
        entries[0] = horizontal
        entries[1] = vertical
        setLimelightNTDoubleArray(limelightName, "keystone_set", entries)
    }

    /**
     * Sets 3D offset point for easy 3D targeting.
     */
    fun setFiducial3DOffset(limelightName: String?, offsetX: Double, offsetY: Double, offsetZ: Double) {
        val entries = DoubleArray(3)
        entries[0] = offsetX
        entries[1] = offsetY
        entries[2] = offsetZ
        setLimelightNTDoubleArray(limelightName, "fiducial_offset_set", entries)
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     *
     * @param limelightName Name/identifier of the Limelight
     * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
     * @param pitch (Unnecessary) Robot pitch in degrees
     * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
     * @param roll (Unnecessary) Robot roll in degrees
     * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
     */
    fun SetRobotOrientation(
        limelightName: String, yaw: Double, yawRate: Double,
        pitch: Double, pitchRate: Double,
        roll: Double, rollRate: Double
    ) {
        SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, true)
    }

    fun SetRobotOrientation_NoFlush(
        limelightName: String, yaw: Double, yawRate: Double,
        pitch: Double, pitchRate: Double,
        roll: Double, rollRate: Double
    ) {
        SetRobotOrientation_INTERNAL(limelightName, yaw, yawRate, pitch, pitchRate, roll, rollRate, false)
    }

    private fun SetRobotOrientation_INTERNAL(
        limelightName: String, yaw: Double, yawRate: Double,
        pitch: Double, pitchRate: Double,
        roll: Double, rollRate: Double, flush: Boolean
    ) {
        val entries = DoubleArray(6)
        entries[0] = yaw
        entries[1] = yawRate
        entries[2] = pitch
        entries[3] = pitchRate
        entries[4] = roll
        entries[5] = rollRate
        setLimelightNTDoubleArray(limelightName, "robot_orientation_set", entries)
        if (flush) {
            Flush()
        }
    }

    /**
     * Configures the IMU mode for MegaTag2 Localization
     *
     * @param limelightName Name/identifier of the Limelight
     * @param mode IMU mode.
     */
    fun SetIMUMode(limelightName: String?, mode: Int) {
        setLimelightNTDouble(limelightName, "imumode_set", mode.toDouble())
    }

    /**
     * Configures the complementary filter alpha value for IMU Assist Modes (Modes 3 and 4)
     *
     * @param limelightName Name/identifier of the Limelight
     * @param alpha Defaults to .001. Higher values will cause the internal IMU to converge onto the assist source more rapidly.
     */
    fun SetIMUAssistAlpha(limelightName: String?, alpha: Double) {
        setLimelightNTDouble(limelightName, "imuassistalpha_set", alpha)
    }


    /**
     * Configures the throttle value. Set to 100-200 while disabled to reduce thermal output/temperature.
     *
     * @param limelightName Name/identifier of the Limelight
     * @param throttle Defaults to 0. Your Limelgiht will process one frame after skipping <throttle> frames.
    </throttle> */
    fun SetThrottle(limelightName: String?, throttle: Int) {
        setLimelightNTDouble(limelightName, "throttle_set", throttle.toDouble())
    }

    /**
     * Overrides the valid AprilTag IDs that will be used for localization.
     * Tags not in this list will be ignored for robot pose estimation.
     *
     * @param limelightName Name/identifier of the Limelight
     * @param validIDs Array of valid AprilTag IDs to track
     */
    fun SetFiducialIDFiltersOverride(limelightName: String?, validIDs: IntArray) {
        val validIDsDouble = DoubleArray(validIDs.size)
        for (i in validIDs.indices) {
            validIDsDouble[i] = validIDs[i].toDouble()
        }
        setLimelightNTDoubleArray(limelightName, "fiducial_id_filters_set", validIDsDouble)
    }

    /**
     * Sets the downscaling factor for AprilTag detection.
     * Increasing downscale can improve performance at the cost of potentially reduced detection range.
     *
     * @param limelightName Name/identifier of the Limelight
     * @param downscale Downscale factor. Valid values: 1.0 (no downscale), 1.5, 2.0, 3.0, 4.0. Set to 0 for pipeline control.
     */
    fun SetFiducialDownscalingOverride(limelightName: String?, downscale: Float) {
        var d = 0 // pipeline
        if (downscale.toDouble() == 1.0) {
            d = 1
        }
        if (downscale.toDouble() == 1.5) {
            d = 2
        }
        if (downscale == 2f) {
            d = 3
        }
        if (downscale == 3f) {
            d = 4
        }
        if (downscale == 4f) {
            d = 5
        }
        setLimelightNTDouble(limelightName, "fiducial_downscale_set", d.toDouble())
    }

    /**
     * Sets the camera pose relative to the robot.
     * @param limelightName Name of the Limelight camera
     * @param forward Forward offset in meters
     * @param side Side offset in meters
     * @param up Up offset in meters
     * @param roll Roll angle in degrees
     * @param pitch Pitch angle in degrees
     * @param yaw Yaw angle in degrees
     */
    fun setCameraPose_RobotSpace(
        limelightName: String?,
        forward: Double,
        side: Double,
        up: Double,
        roll: Double,
        pitch: Double,
        yaw: Double
    ) {
        val entries = DoubleArray(6)
        entries[0] = forward
        entries[1] = side
        entries[2] = up
        entries[3] = roll
        entries[4] = pitch
        entries[5] = yaw
        setLimelightNTDoubleArray(limelightName, "camerapose_robotspace_set", entries)
    }

    /////
    /////
    fun setPythonScriptData(limelightName: String?, outgoingPythonData: DoubleArray?) {
        setLimelightNTDoubleArray(limelightName, "llrobot", outgoingPythonData)
    }

    fun getPythonScriptData(limelightName: String?): DoubleArray {
        return getLimelightNTDoubleArray(limelightName, "llpython")
    }

    /////
    /////
    /**
     * Triggers a snapshot capture via NetworkTables by incrementing the snapshot counter.
     * Rate-limited to once per 10 frames on the Limelight.
     * @param limelightName Name of the Limelight camera
     */
    fun triggerSnapshot(limelightName: String?) {
        val current = getLimelightNTDouble(limelightName, "snapshot")
        setLimelightNTDouble(limelightName, "snapshot", current + 1)
    }

    /**
     * Enables or pauses the rewind buffer recording.
     * @param limelightName Name of the Limelight camera
     * @param enabled True to enable recording, false to pause
     */
    fun setRewindEnabled(limelightName: String?, enabled: Boolean) {
        setLimelightNTDouble(limelightName, "rewind_enable_set", (if (enabled) 1 else 0).toDouble())
    }

    /**
     * Triggers a rewind capture with the specified duration.
     * Maximum duration is 165 seconds. Rate-limited on the Limelight.
     * @param limelightName Name of the Limelight camera
     * @param durationSeconds Duration of rewind capture in seconds (max 165)
     */
    fun triggerRewindCapture(limelightName: String?, durationSeconds: Double) {
        val currentArray = getLimelightNTDoubleArray(limelightName, "capture_rewind")
        val counter = if ((currentArray.size > 0)) currentArray[0] else 0.0
        val entries = DoubleArray(2)
        entries[0] = counter + 1
        entries[1] = min(durationSeconds, 165.0)
        setLimelightNTDoubleArray(limelightName, "capture_rewind", entries)
    }

    /**
     * Gets the latest JSON results output and returns a LimelightResults object.
     * @param limelightName Name of the Limelight camera
     * @return LimelightResults object containing all current target data
     */
    fun getLatestResults(limelightName: String?): LimelightResults {
        val start = System.nanoTime()
        var results = LimelightResults()
        if (mapper == null) {
            mapper = ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
        }

        try {
            val jsonString = getJSONDump(limelightName)
            if (jsonString == null || jsonString.isEmpty() || jsonString.isBlank()) {
                results.error = "lljson error: empty json"
            } else {
                results = mapper!!.readValue(jsonString, LimelightResults::class.java)
                if (results.imuResults != null) {
                    results.imuResults!!.parseDataArray()
                }
            }
        } catch (e: JsonProcessingException) {
            results.error = "lljson error: " + e.message
        }

        val end = System.nanoTime()
        val millis = (end - start) * .000001
        results.latency_jsonParse = millis
        if (profileJSON) {
            System.out.printf("lljson: %.2f\r\n", millis)
        }

        return results
    }

    /**
     * Sets up port forwarding for a Limelight 3A/3G connected via USB.
     * This allows access to the Limelight web interface and video stream
     * when connected to the robot over USB.
     *
     * For usbIndex 0: ports 5800-5809 forward to 172.29.0.1
     * For usbIndex 1: ports 5810-5819 forward to 172.29.1.1
     * etc.
     *
     * Call this method once during robot initialization.
     * To access the interface of the camera with usbIndex0, you would go to roboRIO-(teamnum)-FRC.local:5801. Port 5811 for usb index 1
     *
     * @param usbIndex The USB index of the Limelight (0, 1, 2, etc.)
     */
    fun setupPortForwardingUSB(usbIndex: Int) {
        val ip = "172.29.$usbIndex.1"
        val basePort = 5800 + (usbIndex * 10)

        for (i in 0..9) {
            PortForwarder.add(basePort + i, ip, 5800 + i)
        }
    }

    /**
     * Represents a Color/Retroreflective Target Result extracted from JSON Output
     */
    class LimelightTarget_Retro {
        @JsonProperty("t6c_ts")
        private val cameraPose_TargetSpace = DoubleArray(6)

        @JsonProperty("t6r_fs")
        private val robotPose_FieldSpace = DoubleArray(6)

        @JsonProperty("t6r_ts")
        private val robotPose_TargetSpace = DoubleArray(6)

        @JsonProperty("t6t_cs")
        private val targetPose_CameraSpace = DoubleArray(6)

        @JsonProperty("t6t_rs")
        private val targetPose_RobotSpace = DoubleArray(6)

        fun getCameraPose_TargetSpace(): Pose3d {
            return toPose3D(cameraPose_TargetSpace)
        }

        fun getRobotPose_FieldSpace(): Pose3d {
            return toPose3D(robotPose_FieldSpace)
        }

        fun getRobotPose_TargetSpace(): Pose3d {
            return toPose3D(robotPose_TargetSpace)
        }

        fun getTargetPose_CameraSpace(): Pose3d {
            return toPose3D(targetPose_CameraSpace)
        }

        fun getTargetPose_RobotSpace(): Pose3d {
            return toPose3D(targetPose_RobotSpace)
        }

        val cameraPose_TargetSpace2D: Pose2d
            get() = toPose2D(cameraPose_TargetSpace)
        val robotPose_FieldSpace2D: Pose2d
            get() = toPose2D(robotPose_FieldSpace)
        val robotPose_TargetSpace2D: Pose2d
            get() = toPose2D(robotPose_TargetSpace)
        val targetPose_CameraSpace2D: Pose2d
            get() = toPose2D(targetPose_CameraSpace)
        val targetPose_RobotSpace2D: Pose2d
            get() = toPose2D(targetPose_RobotSpace)

        @JsonProperty("ta")
        var ta: Double = 0.0

        @JsonProperty("tx")
        var tx: Double = 0.0

        @JsonProperty("ty")
        var ty: Double = 0.0

        @JsonProperty("txp")
        var tx_pixels: Double = 0.0

        @JsonProperty("typ")
        var ty_pixels: Double = 0.0

        @JsonProperty("tx_nocross")
        var tx_nocrosshair: Double = 0.0

        @JsonProperty("ty_nocross")
        var ty_nocrosshair: Double = 0.0

        @JsonProperty("ts")
        var ts: Double = 0.0
    }

    /**
     * Represents an AprilTag/Fiducial Target Result extracted from JSON Output
     */
    class LimelightTarget_Fiducial {
        @JsonProperty("fID")
        var fiducialID: Double = 0.0

        @JsonProperty("fam")
        var fiducialFamily: String? = null

        @JsonProperty("t6c_ts")
        private val cameraPose_TargetSpace = DoubleArray(6)

        @JsonProperty("t6r_fs")
        private val robotPose_FieldSpace = DoubleArray(6)

        @JsonProperty("t6r_ts")
        private val robotPose_TargetSpace = DoubleArray(6)

        @JsonProperty("t6t_cs")
        private val targetPose_CameraSpace = DoubleArray(6)

        @JsonProperty("t6t_rs")
        private val targetPose_RobotSpace = DoubleArray(6)

        fun getCameraPose_TargetSpace(): Pose3d {
            return toPose3D(cameraPose_TargetSpace)
        }

        fun getRobotPose_FieldSpace(): Pose3d {
            return toPose3D(robotPose_FieldSpace)
        }

        fun getRobotPose_TargetSpace(): Pose3d {
            return toPose3D(robotPose_TargetSpace)
        }

        fun getTargetPose_CameraSpace(): Pose3d {
            return toPose3D(targetPose_CameraSpace)
        }

        fun getTargetPose_RobotSpace(): Pose3d {
            return toPose3D(targetPose_RobotSpace)
        }

        val cameraPose_TargetSpace2D: Pose2d
            get() = toPose2D(cameraPose_TargetSpace)
        val robotPose_FieldSpace2D: Pose2d
            get() = toPose2D(robotPose_FieldSpace)
        val robotPose_TargetSpace2D: Pose2d
            get() = toPose2D(robotPose_TargetSpace)
        val targetPose_CameraSpace2D: Pose2d
            get() = toPose2D(targetPose_CameraSpace)
        val targetPose_RobotSpace2D: Pose2d
            get() = toPose2D(targetPose_RobotSpace)

        @JsonProperty("ta")
        var ta: Double = 0.0

        @JsonProperty("tx")
        var tx: Double = 0.0

        @JsonProperty("ty")
        var ty: Double = 0.0

        @JsonProperty("txp")
        var tx_pixels: Double = 0.0

        @JsonProperty("typ")
        var ty_pixels: Double = 0.0

        @JsonProperty("tx_nocross")
        var tx_nocrosshair: Double = 0.0

        @JsonProperty("ty_nocross")
        var ty_nocrosshair: Double = 0.0

        @JsonProperty("ts")
        var ts: Double = 0.0
    }

    /**
     * Represents a Barcode Target Result extracted from JSON Output
     */
    class LimelightTarget_Barcode {
        /**
         * Barcode family type (e.g. "QR", "DataMatrix", etc.)
         */
        @JsonProperty("fam")
        var family: String? = null

        /**
         * Gets the decoded data content of the barcode
         */
        @JsonProperty("data")
        var data: String? = null

        @JsonProperty("txp")
        var tx_pixels: Double = 0.0

        @JsonProperty("typ")
        var ty_pixels: Double = 0.0

        @JsonProperty("tx")
        var tx: Double = 0.0

        @JsonProperty("ty")
        var ty: Double = 0.0

        @JsonProperty("tx_nocross")
        var tx_nocrosshair: Double = 0.0

        @JsonProperty("ty_nocross")
        var ty_nocrosshair: Double = 0.0

        @JsonProperty("ta")
        var ta: Double = 0.0

//        @JsonProperty("pts")
//        var corners: Array<DoubleArray>
    }

    /**
     * Represents a Neural Classifier Pipeline Result extracted from JSON Output
     */
    class LimelightTarget_Classifier {
        @JsonProperty("class")
        var className: String? = null

        @JsonProperty("classID")
        var classID: Double = 0.0

        @JsonProperty("conf")
        var confidence: Double = 0.0

        @JsonProperty("zone")
        var zone: Double = 0.0

        @JsonProperty("tx")
        var tx: Double = 0.0

        @JsonProperty("txp")
        var tx_pixels: Double = 0.0

        @JsonProperty("ty")
        var ty: Double = 0.0

        @JsonProperty("typ")
        var ty_pixels: Double = 0.0
    }

    /**
     * Represents a Neural Detector Pipeline Result extracted from JSON Output
     */
    class LimelightTarget_Detector {
        @JsonProperty("class")
        var className: String? = null

        @JsonProperty("classID")
        var classID: Double = 0.0

        @JsonProperty("conf")
        var confidence: Double = 0.0

        @JsonProperty("ta")
        var ta: Double = 0.0

        @JsonProperty("tx")
        var tx: Double = 0.0

        @JsonProperty("ty")
        var ty: Double = 0.0

        @JsonProperty("txp")
        var tx_pixels: Double = 0.0

        @JsonProperty("typ")
        var ty_pixels: Double = 0.0

        @JsonProperty("tx_nocross")
        var tx_nocrosshair: Double = 0.0

        @JsonProperty("ty_nocross")
        var ty_nocrosshair: Double = 0.0
    }

    /**
     * Represents hardware statistics from the Limelight.
     */
    class HardwareReport {
        @JsonProperty("cid")
        var cameraId: String? = null

        @JsonProperty("cpu")
        var cpuUsage: Double = 0.0

        @JsonProperty("dfree")
        var diskFree: Double = 0.0

        @JsonProperty("dtot")
        var diskTotal: Double = 0.0

        @JsonProperty("ram")
        var ramUsage: Double = 0.0

        @JsonProperty("temp")
        var temperature: Double = 0.0
    }

    /**
     * Represents IMU data from the JSON results.
     */
    class IMUResults {
        @JsonProperty("data")
        var data: DoubleArray? = DoubleArray(0)

        @JsonProperty("quat")
        var quaternion: DoubleArray = DoubleArray(4)

        @JsonProperty("yaw")
        var yaw: Double = 0.0

        // Parsed from data array
        var robotYaw: Double = 0.0
        var roll: Double = 0.0
        var pitch: Double = 0.0
        var rawYaw: Double = 0.0
        var gyroZ: Double = 0.0
        var gyroX: Double = 0.0
        var gyroY: Double = 0.0
        var accelZ: Double = 0.0
        var accelX: Double = 0.0
        var accelY: Double = 0.0

        fun parseDataArray() {
            if (data != null && data!!.size >= 10) {
                robotYaw = data!![0]
                roll = data!![1]
                pitch = data!![2]
                rawYaw = data!![3]
                gyroZ = data!![4]
                gyroX = data!![5]
                gyroY = data!![6]
                accelZ = data!![7]
                accelX = data!![8]
                accelY = data!![9]
            }
        }
    }

    /**
     * Represents capture rewind buffer statistics.
     */
    class RewindStats {
        @JsonProperty("bufferUsage")
        var bufferUsage: Double = 0.0

        @JsonProperty("enabled")
        var enabled: Int = 0

        @JsonProperty("flushing")
        var flushing: Int = 0

        @JsonProperty("frameCount")
        var frameCount: Int = 0

        @JsonProperty("latpen")
        var latencyPenalty: Int = 0

        @JsonProperty("storedSeconds")
        var storedSeconds: Double = 0.0
    }

    /**
     * Limelight Results object, parsed from a Limelight's JSON results output.
     */
    class LimelightResults {
        var error: String? = null

        @JsonProperty("pID")
        var pipelineID: Double = 0.0

        @JsonProperty("tl")
        var latency_pipeline: Double = 0.0

        @JsonProperty("cl")
        var latency_capture: Double = 0.0

        var latency_jsonParse: Double = 0.0

        @JsonProperty("ts")
        var timestamp_LIMELIGHT_publish: Double = 0.0

        @JsonProperty("ts_rio")
        var timestamp_RIOFPGA_capture: Double = 0.0

        @JsonProperty("ts_nt")
        var timestamp_nt: Long = 0

        @JsonProperty("ts_sys")
        var timestamp_sys: Long = 0

        @JsonProperty("ts_us")
        var timestamp_us: Long = 0

        @JsonProperty("v")
        @JsonFormat(shape = JsonFormat.Shape.NUMBER)
        var valid: Boolean = false

        @JsonProperty("pTYPE")
        var pipelineType: String = ""

        @JsonProperty("tx")
        var tx: Double = 0.0

        @JsonProperty("ty")
        var ty: Double = 0.0

        @JsonProperty("txnc")
        var tx_nocrosshair: Double = 0.0

        @JsonProperty("tync")
        var ty_nocrosshair: Double = 0.0

        @JsonProperty("ta")
        var ta: Double = 0.0

        @JsonProperty("botpose")
        var botpose: DoubleArray = DoubleArray(6)

        @JsonProperty("botpose_wpired")
        var botpose_wpired: DoubleArray = DoubleArray(6)

        @JsonProperty("botpose_wpiblue")
        var botpose_wpiblue: DoubleArray = DoubleArray(6)

        @JsonProperty("botpose_tagcount")
        var botpose_tagcount: Double = 0.0

        @JsonProperty("botpose_span")
        var botpose_span: Double = 0.0

        @JsonProperty("botpose_avgdist")
        var botpose_avgdist: Double = 0.0

        @JsonProperty("botpose_avgarea")
        var botpose_avgarea: Double = 0.0

        @JsonProperty("botpose_orb")
        var botpose_orb: DoubleArray = DoubleArray(6)

        @JsonProperty("botpose_orb_wpiblue")
        var botpose_orb_wpiblue: DoubleArray = DoubleArray(6)

        @JsonProperty("botpose_orb_wpired")
        var botpose_orb_wpired: DoubleArray = DoubleArray(6)

        @JsonProperty("t6c_rs")
        var camerapose_robotspace: DoubleArray = DoubleArray(6)

        @JsonProperty("hw")
        var hardware: HardwareReport? = null

        @JsonProperty("imu")
        var imuResults: IMUResults? = null

        @JsonProperty("rewind")
        var rewindStats: RewindStats? = null

        @JsonProperty("PythonOut")
        var pythonOutput: DoubleArray = DoubleArray(0)

        val botPose3d: Pose3d
            get() = toPose3D(botpose)

        val botPose3d_wpiRed: Pose3d
            get() = toPose3D(botpose_wpired)

        val botPose3d_wpiBlue: Pose3d
            get() = toPose3D(botpose_wpiblue)

        val botPose2d: Pose2d
            get() = toPose2D(botpose)

        val botPose2d_wpiRed: Pose2d
            get() = toPose2D(botpose_wpired)

        val botPose2d_wpiBlue: Pose2d
            get() = toPose2D(botpose_wpiblue)

        @JsonProperty("Retro")
        var targets_Retro: Array<LimelightTarget_Retro?> = arrayOfNulls(0)

        @JsonProperty("Fiducial")
        var targets_Fiducials: Array<LimelightTarget_Fiducial?> = arrayOfNulls(0)

        @JsonProperty("Classifier")
        var targets_Classifier: Array<LimelightTarget_Classifier?> = arrayOfNulls(0)

        @JsonProperty("Detector")
        var targets_Detector: Array<LimelightTarget_Detector?> = arrayOfNulls(0)

        @JsonProperty("Barcode")
        var targets_Barcode: Array<LimelightTarget_Barcode?> = arrayOfNulls(0)
    }

    /**
     * Represents a Limelight Raw Fiducial result from Limelight's NetworkTables output.
     */
    class RawFiducial(
        id: Int,
        txnc: Double,
        tync: Double,
        ta: Double,
        distToCamera: Double,
        distToRobot: Double,
        ambiguity: Double
    ) {
        var id: Int = 0
        var txnc: Double = 0.0
        var tync: Double = 0.0
        var ta: Double = 0.0
        var distToCamera: Double = 0.0
        var distToRobot: Double = 0.0
        var ambiguity: Double = 0.0


        init {
            this.id = id
            this.txnc = txnc
            this.tync = tync
            this.ta = ta
            this.distToCamera = distToCamera
            this.distToRobot = distToRobot
            this.ambiguity = ambiguity
        }

        override fun equals(obj: Any?): Boolean {
            if (this === obj) return true
            if (obj == null || javaClass != obj.javaClass) return false
            val other = obj as RawFiducial
            return id == other.id && java.lang.Double.compare(txnc, other.txnc) == 0 && java.lang.Double.compare(
                tync,
                other.tync
            ) == 0 && java.lang.Double.compare(ta, other.ta) == 0 && java.lang.Double.compare(
                distToCamera,
                other.distToCamera
            ) == 0 && java.lang.Double.compare(distToRobot, other.distToRobot) == 0 && java.lang.Double.compare(
                ambiguity,
                other.ambiguity
            ) == 0
        }
    }

    /**
     * Represents a Limelight Raw Target/Contour result from Limelight's NetworkTables output.
     */
    class RawTarget(txnc: Double, tync: Double, ta: Double) {
        var txnc: Double = 0.0
        var tync: Double = 0.0
        var ta: Double = 0.0

        init {
            this.txnc = txnc
            this.tync = tync
            this.ta = ta
        }

        override fun equals(obj: Any?): Boolean {
            if (this === obj) return true
            if (obj == null || javaClass != obj.javaClass) return false
            val other = obj as RawTarget
            return java.lang.Double.compare(txnc, other.txnc) == 0 && java.lang.Double.compare(
                tync,
                other.tync
            ) == 0 && java.lang.Double.compare(ta, other.ta) == 0
        }
    }

    /**
     * Represents a Limelight Raw Neural Detector result from Limelight's NetworkTables output.
     */
    class RawDetection(
        classId: Int, txnc: Double, tync: Double, ta: Double,
        corner0_X: Double, corner0_Y: Double,
        corner1_X: Double, corner1_Y: Double,
        corner2_X: Double, corner2_Y: Double,
        corner3_X: Double, corner3_Y: Double
    ) {
        var classId: Int = 0
        var txnc: Double = 0.0
        var tync: Double = 0.0
        var ta: Double = 0.0
        var corner0_X: Double = 0.0
        var corner0_Y: Double = 0.0
        var corner1_X: Double = 0.0
        var corner1_Y: Double = 0.0
        var corner2_X: Double = 0.0
        var corner2_Y: Double = 0.0
        var corner3_X: Double = 0.0
        var corner3_Y: Double = 0.0


        init {
            this.classId = classId
            this.txnc = txnc
            this.tync = tync
            this.ta = ta
            this.corner0_X = corner0_X
            this.corner0_Y = corner0_Y
            this.corner1_X = corner1_X
            this.corner1_Y = corner1_Y
            this.corner2_X = corner2_X
            this.corner2_Y = corner2_Y
            this.corner3_X = corner3_X
            this.corner3_Y = corner3_Y
        }
    }

    /**
     * Represents a 3D Pose Estimate.
     */
    class PoseEstimate {
        var pose: Pose2d
        var timestampSeconds: Double
        var latency: Double
        var tagCount: Int
        var tagSpan: Double
        var avgTagDist: Double
        var avgTagArea: Double

        var rawFiducials: Array<RawFiducial?>?
        var isMegaTag2: Boolean

        /**
         * Instantiates a PoseEstimate object with default values
         */
        constructor() {
            this.pose = Pose2d()
            this.timestampSeconds = 0.0
            this.latency = 0.0
            this.tagCount = 0
            this.tagSpan = 0.0
            this.avgTagDist = 0.0
            this.avgTagArea = 0.0
            this.rawFiducials = arrayOf()
            this.isMegaTag2 = false
        }

        constructor(
            pose: Pose2d, timestampSeconds: Double, latency: Double,
            tagCount: Int, tagSpan: Double, avgTagDist: Double,
            avgTagArea: Double, rawFiducials: Array<RawFiducial?>?, isMegaTag2: Boolean
        ) {
            this.pose = pose
            this.timestampSeconds = timestampSeconds
            this.latency = latency
            this.tagCount = tagCount
            this.tagSpan = tagSpan
            this.avgTagDist = avgTagDist
            this.avgTagArea = avgTagArea
            this.rawFiducials = rawFiducials
            this.isMegaTag2 = isMegaTag2
        }

        override fun equals(obj: Any?): Boolean {
            if (this === obj) return true
            if (obj == null || javaClass != obj.javaClass) return false
            val that = obj as PoseEstimate
            // We don't compare the timestampSeconds as it isn't relevant for equality and makes
            // unit testing harder
            return java.lang.Double.compare(
                that.latency,
                latency
            ) == 0 && tagCount == that.tagCount && java.lang.Double.compare(
                that.tagSpan,
                tagSpan
            ) == 0 && java.lang.Double.compare(
                that.avgTagDist,
                avgTagDist
            ) == 0 && java.lang.Double.compare(that.avgTagArea, avgTagArea) == 0 && pose == that.pose
                    && rawFiducials.contentEquals(that.rawFiducials)
        }
    }

    /**
     * Encapsulates the state of an internal Limelight IMU.
     */
    class IMUData {
        var robotYaw: Double = 0.0
        var Roll: Double = 0.0
        var Pitch: Double = 0.0
        var Yaw: Double = 0.0
        var gyroX: Double = 0.0
        var gyroY: Double = 0.0
        var gyroZ: Double = 0.0
        var accelX: Double = 0.0
        var accelY: Double = 0.0
        var accelZ: Double = 0.0

        constructor()

        constructor(imuData: DoubleArray?) {
            if (imuData != null && imuData.size >= 10) {
                this.robotYaw = imuData[0]
                this.Roll = imuData[1]
                this.Pitch = imuData[2]
                this.Yaw = imuData[3]
                this.gyroX = imuData[4]
                this.gyroY = imuData[5]
                this.gyroZ = imuData[6]
                this.accelX = imuData[7]
                this.accelY = imuData[8]
                this.accelZ = imuData[9]
            }
        }
    }
}