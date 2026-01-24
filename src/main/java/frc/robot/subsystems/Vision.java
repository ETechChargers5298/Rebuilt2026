package frc.robot.subsystems;


import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.AprilCam;

import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Vision extends SubsystemBase {

  private static Vision instance;
  public AprilCam cam1;
  public AprilCam cam2;
  public boolean doubleCam = true;
  public int closestId;
  Drivetrain drivetrain = Drivetrain.getInstance();
  private final SendableChooser<Integer> tagChooser = new SendableChooser<>();

  // Vision Constructor
  private Vision() {
    
    // Construct each AprilCam
    this.cam1 = new AprilCam(
      VisionConstants.CAM1_NAME, 
      VisionConstants.CAM1_POSITION_OFFSET, 
      VisionConstants.CAM1_ANGLE_OFFSET
    );
    
    // Update the cameras
    cam1.update();
    
    // Option to add 2nd camera
    if(doubleCam){
      this.cam2 = new AprilCam(
        VisionConstants.CAM2_NAME, 
        VisionConstants.CAM2_POSITION_OFFSET, 
        VisionConstants.CAM2_ANGLE_OFFSET
      );
      cam2.update();  
    }

    for(int i=1; i<=22; i++){
      tagChooser.addOption("AT "+i, i);
    }
    
  }

  // Camera Singleton - ensures only one Camera instance is constructed
  public static Vision getInstance(){
    if(instance == null){
      instance = new Vision();
    }
      return instance;
  }

  
  public PhotonTrackedTarget getDesiredTarget(int target) {
    return cam1.getDesiredTarget(target);
  }

  public double getXDesired(PhotonTrackedTarget target) {
    return cam1.getXDesired(target);
  }

  public  Pose3d getTagPose(int tagId){
    return FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get();
  }
  
  public int getClosestId(){
    // return cam1.closestId;
    closestId = FieldConstants.getNearestReefTag(new Pose3d(drivetrain.getPose()));
    return closestId;
  }  


  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    cam1.update();
    // Correct pose estimate with vision measurements
    var visionEst1 = cam1.getEstimatedGlobalPose(drivetrain.getPose());
    visionEst1.ifPresent(
      est -> {
        // Change our trust in the measurement based on the tags we can see
        var estimatedSDs = cam1.getEstimationSDs();

        drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estimatedSDs);
      }
    );
    if(visionEst1.isPresent()) {
      SmartDashboard.putNumber("cam1 poseX", visionEst1.get().estimatedPose.getX());
      SmartDashboard.putNumber("cam1 poseY", visionEst1.get().estimatedPose.getY());
      SmartDashboard.putNumber("cam1 poseRot", visionEst1.get().estimatedPose.getRotation().getAngle());
    }

    if(doubleCam){
      cam2.update();
      var visionEst2 = cam2.getEstimatedGlobalPose(drivetrain.getPose());
      visionEst2.ifPresent(
        est -> {
          var estimatedSDs = cam2.getEstimationSDs();
          drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estimatedSDs);
        }
      );
      if(visionEst2.isPresent()) {
        SmartDashboard.putNumber("cam2 poseX", visionEst2.get().estimatedPose.getX());
        SmartDashboard.putNumber("cam2 poseY", visionEst2.get().estimatedPose.getY());
        SmartDashboard.putNumber("cam2 poseRot", visionEst2.get().estimatedPose.getRotation().getAngle());
      }
    }

    // int tagId = tagChooser.getSelected();
    int tagId = getClosestId();

    if(tagId > 0) {
      SmartDashboard.putNumber("tag " + tagId + " pose x", FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get().getX());
      SmartDashboard.putNumber("tag " + tagId + " pose y", FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get().getY());
      SmartDashboard.putNumber("tag " + tagId + " angle", FieldConstants.aprilTagFieldLayout.getTagPose(tagId).get().getRotation().getAngle());
    }

    SmartDashboard.putNumber("closest Id", getClosestId());
    SmartDashboard.putNumber("closest x", getXDesired(getDesiredTarget(closestId)));

    SmartDashboard.putNumber("CAM1 X offset to Front", Constants.VisionConstants.CAM1_X_OFFSET_TO_FRONT);
    SmartDashboard.putNumber("CAM1 X offset to Center", Constants.VisionConstants.CAM1_X_OFFSET_TO_CENTER);
    SmartDashboard.putNumber("CAM1 Bumper to Center Dist", Constants.RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE);
    
    
  }
}