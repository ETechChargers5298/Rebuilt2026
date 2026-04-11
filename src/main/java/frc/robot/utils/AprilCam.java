package frc.robot.utils;


import frc.robot.FieldConstants;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class AprilCam {

    // April 10, 2026: Code changes made to reflect Photon example at https://github.com/PhotonVision/photonvision/blob/a5be3d062ce5c145f4a759b0207057f218cda6b3/photonlib-java-examples/poseest/src/main/java/frc/robot/Vision.java

    private PhotonCamera camera;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;

    public int closestId;
    public double closestDistance;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private Matrix<N3, N1> curStdDevs;
    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
    
    // Constructor 1
    public AprilCam(String name, Translation3d pos, Rotation3d angle){
        this.camera = new PhotonCamera(name);
        this.camOffset = new Transform3d(pos, angle);
        this.photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, camOffset);
     }

     // Constructor 2: simple version
    public AprilCam(String name) {
        this(name, new Translation3d(), new Rotation3d());
    }


    // ---------------- UPDATES ----------------------------------//

    public void update(EstimateConsumer consumer) { // should get called exactly ONCE per robot loop
        List<PhotonTrackedTarget> latestTargets = new ArrayList<>();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = photonPoseEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonPoseEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionEst.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs();

                        consumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });

            
            latestTargets = result.getTargets();
        }

        updateClosestId(latestTargets);

        // Display each target info seen
        SmartDashboard.putNumber("ClosestID", closestId);
        for (int i = 0; latestTargets != null && i < latestTargets.size(); i++) {
          SmartDashboard.putString("Id" + i, latestTargets.get(i).toString());
        }
    }

    // Private method that occurs in the update
    private void updateClosestId(List<PhotonTrackedTarget> targets) {

        // Stop trying if there are no targets visible
        if(targets == null || targets.isEmpty()) return;

        // Initialize variable to find best ID & distance
        int closestId = -1;
        double minDistance = Double.MAX_VALUE;
        
        // Loop through all the targets
        for(PhotonTrackedTarget t: targets) {
        
            // Calculate distance between the camera and the target
            double checkDistance = Math.sqrt(Math.pow(getTargetTransform(t).getX(), 2) + Math.pow(getTargetTransform(t).getY(), 2));

            // Update the minDistance if needed
            if(checkDistance < minDistance){
                minDistance = checkDistance;
                closestId = t.fiducialId;
            }
        }

        // If there was a closestId found, update the fields
        if (closestId != -1){
            this.closestId = closestId;
            this.closestDistance = minDistance;
        }
    }

    // Gets the Transform3d object of a specific AprilTag object
    private Transform3d getTargetTransform(PhotonTrackedTarget target){
        if(target == null) {
            return new Transform3d();
        }
        return target.getBestCameraToTarget();
    }


    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
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
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }


    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

   

}