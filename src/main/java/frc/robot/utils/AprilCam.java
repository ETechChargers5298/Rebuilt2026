package frc.robot.utils;


import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.FieldConstants;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** Add your docs here. */
public class AprilCam {

    private PhotonCamera camera;
    private Transform3d camOffset;
    private PhotonPoseEstimator photonPoseEstimator;
    private List<PhotonPipelineResult> results;
    private List<EstimatedRobotPose> estimatedPoses;
    private PhotonPipelineResult lastResult;

    private PhotonTrackedTarget desiredTarget;
    private List<PhotonTrackedTarget> targets;
    public int closestId;
    public double closestDistance;
    private PhotonTrackedTarget closestTarget;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;
    private Matrix<N3, N1> currentSDs;
    
    
    // Constructor 1
    public AprilCam(String name, Translation3d pos, Rotation3d angle){
        this.camera = new PhotonCamera(name);
        this.camOffset = new Transform3d(pos, angle);
        this.photonPoseEstimator = new PhotonPoseEstimator(FieldConstants.aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOffset);
        this.estimatedPoses = new ArrayList<EstimatedRobotPose>();
        this.closestTarget = new PhotonTrackedTarget();
        updateEstimationSDs();
     }

     // Constructor 2: simple version
    public AprilCam(String name) {
        this(name,new Translation3d(), new Rotation3d());
    }


    public void update() { // should get called exactly ONCE per robot loop
        
        // Update the List of PhotonPipeline results each cycle
        this.results = camera.getAllUnreadResults();

        // Update the List of PhotonTrackedTargets seen each cycle
        for (var result: results){
            targets = result.getTargets();
            updateClosestId();
        }

        // Update the estimation SDs
        updateEstimationSDs();

        // Display each target info seen
        SmartDashboard.putNumber("ClosestID",closestId);
        SmartDashboard.putNumber("closestX", getXClosest());
        // SmartDashboard.putNumber("closestY", getYClosest());
        // SmartDashboard.putNumber("closestZ", getZclosest());
        for (int i = 0; getTargets() != null && i < getTargets().size(); i++) {
          SmartDashboard.putString("Id" + i, getTargets().get(i).toString());
        }
    }

    // --------------------- GETTING TARGETS -------------------)------------ //

    // Checks if a specific result has a target
    public boolean hasTarget(PhotonPipelineResult result) {
        return result.hasTargets();
    }

    // Checks if any results have targets
    public boolean hasAnyTarget(){
        for(PhotonPipelineResult result: results){
            if(hasTarget(result)){
                return true;
            }
        }
        return false;
    }

    //Gets all the AprilTag targets the camera can currently see
    public List<PhotonTrackedTarget> getTargets(){
        return targets;
    }

    // Private method that occurs in the update
    private void updateClosestId() {

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
                this.closestTarget = t;
            }
        }

        // If there was a closestId found, update the fields
        if (closestId != -1){
            this.closestId = closestId;
            this.closestDistance = minDistance;
        }
    }

    // Gets the current "best" target
    // public PhotonTrackedTarget getBestTarget(){
    //     return result.getBestTarget();
    // }

    // Gets a target object for a specific AprilTag
    public PhotonTrackedTarget getDesiredTarget(int desiredTargetId) {

        //look at each target in the arraylist of targets
        if(getTargets() != null){
            for (PhotonTrackedTarget t: getTargets())
            {
                // look for the target with the desired Id
                 if (t.getFiducialId() == desiredTargetId)
                 {
                    return t;
                 }
                // return t;  // this automatically return the first ID it sees
            
                }
        }
        //return null if you can't find the desiredTarget
        System.out.println("AprilCam cannot find desired target " + desiredTargetId);
        System.out.println("targets " + getTargets());
        return null;
    }

    // Checks if a desired AprilTag is visible
    public boolean hasDesiredTarget(int desiredTargetId) {
        ///use the getDesiredTarget method to see if it returns null (not correct target) or not
        if (getDesiredTarget(desiredTargetId)!= null)
        {
            return true;
        }
        return false;
    }

    // --------------------- GETTING DATA FROM A TARGET ------------------------------- //
    // https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html#getting-data-from-a-target
    // double yaw = target.getYaw();
    // double pitch = target.getPitch();
    // double area = target.getArea();
    // double skew = target.getSkew(); //not available for AprilTags
    // Transform2d pose = target.getCameraToTarget();
    // List<TargetCorner> corners = target.getCorners();
    //  double targetId = target.getfiducialId();
    // double poseAmbiguity = target.getPoseAmbiguity();
    // Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    // Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // Gets the Transform3d object of a specific AprilTag object
    private Transform3d getTargetTransform(PhotonTrackedTarget target){
        if(target == null) {
            System.out.println("No target found");
            return new Transform3d();
        }

        return new Transform3d();
        // return target.getBestCameraToTarget();
    }



    // --------------------- POSE ESTIMATION ------------------------------- //
   
    // The latest estimated robot pose on the field from vision data. This may be empty.
    // This should only be called once per loop
    public List<EstimatedRobotPose> getUpdatedEstPoses(Pose2d prevEstPose) {

        // Create an empty ArrayList to store estimated poses from each tag seen
        List<EstimatedRobotPose> visionEstPoses = new ArrayList<>();

        // Give PhotonVision the drivetrain's current best estimate of Pose
        photonPoseEstimator.setReferencePose(prevEstPose);

        // Loop through all the results of the camera
        for (var result : results) {

            // Get an update from the result
            var estPoseUpdate = photonPoseEstimator.update(result);

            // Add to the List of Estimated Poses
            if (estPoseUpdate.isPresent()) {
                EstimatedRobotPose pose = estPoseUpdate.get();
                visionEstPoses.add(pose);
            }

        }

        this.estimatedPoses = visionEstPoses;
        return visionEstPoses;
    }


    /*
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     * Uses higher weights for reef tags!
     * This should only be used when there are targets visible
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationSDs() {

        // No pose input. Default to single-tag std devs
        if (estimatedPoses.isEmpty()) {
            currentSDs = VisionConstants.SINGLE_TAG_SD;
        } 
        
        // Pose present. Start running Heuristic
        else {
            var estimatedSDs = VisionConstants.SINGLE_TAG_SD;
            int numTags = 0;
            double totalDistance = 0;
            double totalWeight = 0;

            // Loop through all the targets to find difference in distance from current pose & how important that tag is
            for (var target : targets) {
                // Pose3d tagPose = FieldConstants.getTagPose(target.getFiducialId());
                // if(tagPose != null){
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                Translation2d currentTranslation = estimatedPoses.get(0).estimatedPose.toPose2d().getTranslation();  //Drivetrain.getInstance().get  s.get().estimatedPoses.toPose2d().getTranslation()
                totalDistance += tagPose.get().toPose2d().getTranslation().getDistance(currentTranslation);
                totalWeight += FieldConstants.TAG_WEIGHTS[target.getFiducialId() - 1]; 
            }

            // Use the single tag standard deviations if no tags visible
            if (numTags == 0) {
                currentSDs = VisionConstants.SINGLE_TAG_SD;
            } 
            
            // One or more tags visible, run the full heuristic.
            else {

                // Calculate the average distance changes were
                double avgDist = totalDistance /numTags;
                double avgWeight = totalWeight /numTags;

                // Decrease standard deviations if multiple targets are visible
                if (numTags > 1) estimatedSDs = VisionConstants.MULTI_TAG_SD;
                
                // Increase standard deviations based on average distance
                if (numTags == 1 && avgDist > 4){
                    estimatedSDs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estimatedSDs = estimatedSDs.times(1 + (avgDist * avgDist / 30));
                }

                // Weight reef tags higher
                estimatedSDs = estimatedSDs.times(avgWeight);
                currentSDs = estimatedSDs;
            }
        }
    }
     

    /*
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationSDs() {
        return currentSDs;
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    
    //method that gets the Id of the "best" target, generally not used by our robot
    // public String getAllTargets(){

    //     Optional<MultiTargetPNPResult> target = result.getMultiTagResult();

    //     return target.get().fiducialIdsUsed.toString();
    // }


    //---------------------HELPER METHODS -------------------------//
    
    // // Gets the X value of a desired target
    // public double getXDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getX();
    // }

    // Gets the X value of the "Best" target
    public double getXClosest(){
        if(closestTarget == null){ return -1.0;}
        return getTargetTransform(closestTarget).getX();
        // return getXDesired( closestTarget );
    }
    
    // // Gets the Y value of a desired target
    // public double getYDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getY();
    // }

    // // Gets the Y value of the "Best" target
    // public double getYClosest(){
    //     return getYDesired( closestTarget );
    // }

    // // Gets the Z value of a desired target
    // public double getZDesired(PhotonTrackedTarget target){
    //     if(target == null) { return Float.NaN; }
    //     return getTargetTransform(target).getZ();
    // }

    // // Gets the Z value of the "Best" target
    // public double getZclosest(){
    //     return getZDesired( closestTarget);
    // }

   

}