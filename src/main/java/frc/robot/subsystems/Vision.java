package frc.robot.subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.utils.AprilCam;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision extends SubsystemBase {

    // VISION CLASS FIELDS
    private static Vision instance;
    public AprilCam cam1;
    public AprilCam cam2;
    public boolean doubleCam = true;

    public boolean enabled = true;
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

        // Initialize cam2 if desired
        if(doubleCam){
            this.cam2 = new AprilCam(
                VisionConstants.CAM2_NAME,
                VisionConstants.CAM2_POSITION_OFFSET,
                VisionConstants.CAM2_ANGLE_OFFSET
            );
        }
    }

    // VISION SINGLETON
    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    public Command enable(){
        return new InstantCommand(() -> enabled = true);
    } 

    public Command disable(){
        return new InstantCommand(() -> enabled = false);
    }


    @Override
    public void periodic() {
        if(enabled == false) {
            return;
        }
       // Process Camera 1
        cam1.update(drivetrain::addVisionMeasurement);

        // Process Camera 2 (if enabled)
        if (doubleCam) {
            cam2.update(drivetrain::addVisionMeasurement);
        }

    }  // close periodic
        

} // close Vision class
