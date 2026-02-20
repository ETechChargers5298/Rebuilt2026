package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Vision extends SubsystemBase {

    // VISION CLASS FIELDS
    private static Vision instance;
    public AprilCam cam1;
    public AprilCam cam2;
    public boolean doubleCam = false;
    Drivetrain drivetrain = Drivetrain.getInstance();
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);


    // VISION CONSTRUCTOR
    private Vision() {

        // Initialize cam1
        this.cam1 = new AprilCam(
            VisionConstants.CAM1_NAME,
            VisionConstants.CAM1_POSITION_OFFSET,
            VisionConstants.CAM1_ANGLE_OFFSET
        );

        // Get first update from cam1
        cam1.update();

        // Initialize cam2 if desired
        if(doubleCam){
            this.cam2 = new AprilCam(
                VisionConstants.CAM2_NAME,
                VisionConstants.CAM2_POSITION_OFFSET,
                VisionConstants.CAM2_ANGLE_OFFSET
            );
            // Get first update from cam2
            cam2.update();
        }
    }

    // VISION SINGLETON
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }


    @Override
    public void periodic() {

       // Process Camera 1
        processCamera(cam1);

        // Process Camera 2 (if enabled)
        if (doubleCam) {
            processCamera(cam2);
        }

        // DISPLAY STUFF ON SMARTDASHBOARD

        // cam1.hasTarget(cam1.targets)


    }  // close periodic



    private void processCamera(AprilCam cam) {

        //Get the current best guess of the position of the robot based on its wheels
        Pose3d currentRobotPose = new Pose3d(drivetrain.getState().Pose);

        // Ensure cam is up-to-date
        cam.update();

        // Get the latest pose estimates from cam1
        var visionEstPoses1 = cam.getLatestEstimates(currentRobotPose);

        // Loop through all the pose estimate updates from cam1
        for(var update : visionEstPoses1){

            // Get the confidence level from cam
            // var confidence = cam.getEstimationSDs();
            var confidence = edu.wpi.first.math.VecBuilder.fill(0.1,0.1,0.1); // test high trust in vision
            
            // Add each vision measurement update to the drivetrain's pose estimator
            drivetrain.addVisionMeasurement(
                update.estimatedPose.toPose2d(),
                update.timestampSeconds,
                confidence
            );
        }
    }
        


} // close Vision class
