package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.FieldConstants;
import frc.robot.utils.AprilCam;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Vision extends SubsystemBase {

    private static Vision instance;
    
    public AprilCam cam1;
    public AprilCam cam2;
    public boolean doubleCam = false;
    Drivetrain drivetrain = Drivetrain.getInstance();

    private Vision() {
        // Initialize the cameras
        this.cam1 = new AprilCam(
            VisionConstants.CAM1_NAME,
            VisionConstants.CAM1_POSITION_OFFSET,
            VisionConstants.CAM1_ANGLE_OFFSET
            );

            //update cam1
            cam1.update();

        if(doubleCam){
            this.cam2 = new AprilCam(
                VisionConstants.CAM2_NAME,
                VisionConstants.CAM2_POSITION_OFFSET,
                VisionConstants.CAM2_ANGLE_OFFSET
                );
                //update cam2
                cam2.update();
        }
    }
        // Set up the cam SINGLTON instance
        public static Vision getInstance() {
            if (instance == null) {
                instance = new Vision();
            }
            return instance;
        }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //Get the current best guess of the position of the robot based on its wheels
        Pose2d currentRobotPose = drivetrain.getState().Pose;

        //ask the camera for global pose
        cam1.update();
        var VisionEst = cam1.getEstimatedGlobalPose(currentRobotPose);
        
        //if the camera see's a tag, push into drivetrain
        VisionEst.ifPresent(est -> {
            var confidence = cam1.getEstimationSDs();

            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(), 
                est.timestampSeconds, 
                confidence
            );
        });
    }
}
