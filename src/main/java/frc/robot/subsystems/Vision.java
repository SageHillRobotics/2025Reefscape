package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
    private PhotonCamera leftCam;
    private PhotonCamera rightCam;
    private PhotonCamera stationCam;
    private PhotonPoseEstimator photonPoseEstimatorLeft;
    private PhotonPoseEstimator photonPoseEstimatorRight;
    private PhotonPoseEstimator photonPoseEstimatorStation;
    private Matrix<N3, N1> curStdDevsLeft;
    private Matrix<N3, N1> curStdDevsRight;
    private Matrix<N3, N1> curStdDevsStation;



    //Sim
    private VisionSystemSim visionSim;
    private PhotonCameraSim leftReefCameraSim;
    private PhotonCameraSim rightReefCameraSim;
    private PhotonCameraSim stationCameraSim;

    public Vision() {
        leftCam = new PhotonCamera("Left_Reef_Cam");
        rightCam = new PhotonCamera("Right_Reef_Cam");
        stationCam = new PhotonCamera("Coral_Station_Cam");

        photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.LEFT_CAM_TRANSFORM);
        photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.RIGHT_CAM_TRANSFORM);
        photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonPoseEstimatorStation = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.STATION_CAM_TRANSFORM);
        photonPoseEstimatorStation.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation()){
            visionSim = new VisionSystemSim("Vision Sim");
            visionSim.addAprilTags(aprilTagFieldLayout);

            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(30);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);

            leftReefCameraSim = new PhotonCameraSim(leftCam, cameraProp);
            rightReefCameraSim = new PhotonCameraSim(rightCam, cameraProp);
            stationCameraSim = new PhotonCameraSim(stationCam, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(leftReefCameraSim, Constants.VisionConstants.LEFT_CAM_TRANSFORM);
            visionSim.addCamera(rightReefCameraSim, Constants.VisionConstants.RIGHT_CAM_TRANSFORM);
            visionSim.addCamera(stationCameraSim, Constants.VisionConstants.STATION_CAM_TRANSFORM);


            leftReefCameraSim.enableDrawWireframe(true);
            rightReefCameraSim.enableDrawWireframe(true);
            stationCameraSim.enableDrawWireframe(true);
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLeft() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : leftCam.getAllUnreadResults()) {
            visionEst = photonPoseEstimatorLeft.update(change);
            updateEstimationStdDevsLeft(visionEst, change.getTargets());

        }
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseRight() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : rightCam.getAllUnreadResults()) {
            visionEst = photonPoseEstimatorRight.update(change);
            updateEstimationStdDevsRight(visionEst, change.getTargets());

        }
        return visionEst;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseStation() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : stationCam.getAllUnreadResults()) {
            visionEst = photonPoseEstimatorStation.update(change);
            updateEstimationStdDevsStation(visionEst, change.getTargets());

        }
        return visionEst;
    }

    private void updateEstimationStdDevsLeft(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevsLeft = Constants.VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevsLeft = Constants.VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevsLeft = estStdDevs;
            }
        }
    }

    private void updateEstimationStdDevsRight(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevsRight = Constants.VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimatorRight.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevsRight = Constants.VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevsRight = estStdDevs;
            }
        }
    }

    private void updateEstimationStdDevsStation(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevsStation = Constants.VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimatorStation.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevsStation = Constants.VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevsStation = estStdDevs;
            }
        }
    }

    //separate later
    public Matrix<N3, N1> getEstimationStdDevsLeft() {
        return curStdDevsLeft;
    }
    public Matrix<N3, N1> getEstimationStdDevsRight() {
        return curStdDevsRight;
    }

    public Matrix<N3, N1> getEstimationStdDevsStation() {
        return curStdDevsStation;
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    public void simulationPeriodic(Pose2d pose){
        visionSim.update(pose);
    }

    // public List<PhotonTrackedTarget> getTargets(){
    //     if (cam.getLatestResult().hasTargets()){
    //             return (cam.getLatestResult().getTargets());

    //     }
    //     else{
    //         return null;
    //     }
    // }
}