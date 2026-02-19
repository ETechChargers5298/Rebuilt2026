package frc.robot;



// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import com.pathplanner.lib.util.*;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.*;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
// import frc.robot.utils.ModuleConfig;
import frc.robot.Ports;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwerveConstantsOld {
 // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(RobotConstants.WHEEL_BASE / 2, RobotConstants.TRACK_WIDTH / 2),
        new Translation2d(RobotConstants.WHEEL_BASE / 2, -RobotConstants.TRACK_WIDTH / 2),
        new Translation2d(-RobotConstants.WHEEL_BASE / 2, RobotConstants.TRACK_WIDTH / 2),
        new Translation2d(-RobotConstants.WHEEL_BASE / 2, -RobotConstants.TRACK_WIDTH / 2)
    );

    // Is NavX rotation values backwards?
    public static final boolean TURN_INVERSION = false;

    // Driving Parameters - max speeds allowed, not capable
    public static final double TOP_SPEED = 4.0; //9.6
    public static final double TOP_ANGULAR_SPEED = 2 * Math.PI;


  }
  
  
  public static class MechConstants{

    
    public static final int ENCODER_TICKS = 8192; //Counts per Revolution

    // Mech Motor Speeds for Buttons
    public static double CORAL_INTAKE_SPEED = 0.25;
    public static double CORAL_RETRACT_SPEED = 0.3;
    public static double CORAL_SCORE_SPEED = 0.4;
    public static double ALGAE_INTAKE_SPEED = 1.0;

    // Jaw Angles
    public static final double JAW_STARTING_ANGLE = 0;//260;
    public static final double JAW_INTAKE_ANGLE = -34;
    public static final double JAW_UP_ANGLE = -75;//25;
    public static final double JAW_MAX_ANGLE = -80;//25;
    // public static final double JAW_AUTO_ANGLE = 80;

  }

  public static class RobotConstants{
   
    //Robot Chassis Width
    public static final double CHASSIS_WIDTH = Units.inchesToMeters(28);  

    //Bumper thickness
    public static final double BUMPER_WIDTH = Units.inchesToMeters(3); //0.0635 meters 

    //Overall Robot's Width including bumpers
    public static final double ROBOT_OVERALL_WIDTH = BUMPER_WIDTH*2 + CHASSIS_WIDTH;

    // Distance from front of bumper to center of robot
    public static final double BUMPER_TO_ROBOT_CENTER_DISTANCE = ROBOT_OVERALL_WIDTH/2;

    // Distance between centers of left and right wheels on robot
    public static final double TRACK_WIDTH = Units.inchesToMeters(25);

    // Distance between centers of front and back wheels on robot
    public static final double WHEEL_BASE = Units.inchesToMeters(25);

    // Distance from center of any wheel to center of robot geometry
    public static final double WHEEL_TO_CENTER_DISTANCE = Math.sqrt(Math.pow(WHEEL_BASE/2, 2) + Math.pow(TRACK_WIDTH/2, 2));

    // Weight of Robot
    public static final Mass MASS = Kilograms.of(63); //was 25kg = 55lbs, 63kg =140lb
    
    // Moment of Inertia of the Robot, Typical FRC robot will be between 3-8 Kg*m^2
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(5);
  }

  public static class VisionConstants{

    //Camera Name
    public static final String CAM1_NAME = "sparkyCam1";
    public static double CAM1_X_OFFSET_TO_FRONT = -(0.236 + RobotConstants.BUMPER_WIDTH); //cam mounted 12.5" back from front bumper
    public static double CAM1_X_OFFSET_TO_CENTER = CAM1_X_OFFSET_TO_FRONT + RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE;
    public static double CAM1_Y_OFFSET_TO_CENTER = 0; //-0.034; //-0.013;
    public static Translation3d CAM1_POSITION_OFFSET = new Translation3d(CAM1_X_OFFSET_TO_CENTER, CAM1_Y_OFFSET_TO_CENTER,0.0); // is cam mounted at center? how far back from front of bumper?
    public static Rotation3d CAM1_ANGLE_OFFSET = new Rotation3d(0,0,0); // is cam mounted facing forward, upright? 

    public static final String CAM2_NAME = "sparkyCam2"; 
    public static double CAM2_X_OFFSET_TO_CENTER = CAM1_X_OFFSET_TO_CENTER;
    public static double CAM2_Y_OFFSET_TO_CENTER = Units.inchesToMeters(6.5);
    public static Translation3d CAM2_POSITION_OFFSET = new Translation3d(CAM2_X_OFFSET_TO_CENTER, CAM2_Y_OFFSET_TO_CENTER,0.0); // is cam mounted at center? how far back from front of bumper?
    public static Rotation3d CAM2_ANGLE_OFFSET = new Rotation3d(0,0,0); // is cam mounted facing forward, upright? 


    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> SINGLE_TAG_SD = VecBuilder.fill(2, 2, 4);
    public static final Matrix<N3, N1> MULTI_TAG_SD = VecBuilder.fill(0.5, 0.5, 1);

  }

  


}