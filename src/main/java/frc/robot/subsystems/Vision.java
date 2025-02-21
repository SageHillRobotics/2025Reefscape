package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
    private PhotonCamera leftCam;
    private PhotonCamera rightCam;
    private PhotonPoseEstimator photonPoseEstimatorLeft;
    private PhotonPoseEstimator photonPoseEstimatorRight;
    private Matrix<N3, N1> curStdDevs;


    public Vision() {
        leftCam = new PhotonCamera("Left_Reef_Cam");
        rightCam = new PhotonCamera("Right_Reef_Cam");

        photonPoseEstimatorLeft = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.LEFT_CAM_TRANSFORM);
        photonPoseEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        photonPoseEstimatorRight = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Constants.VisionConstants.RIGHT_CAM_TRANSFORM);
        photonPoseEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseLeft() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : leftCam.getAllUnreadResults()) {
            visionEst = photonPoseEstimatorLeft.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets(), photonPoseEstimatorLeft);

        }
        return visionEst;

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPoseRight() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : rightCam.getAllUnreadResults()) {
            visionEst = photonPoseEstimatorRight.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets(), photonPoseEstimatorRight);

        }
        return visionEst;

    }

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator photonPoseEstimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
                curStdDevs = Constants.VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = Constants.VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    //separate later
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
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