// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.Matrix;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class SwerveConstants {
  
  }

  public static final class SwerveModuleConstants {
  
  }
  
  public static class MechConstants{

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

    //Camera 1
    public static final String CAM1_NAME = "SplashyCam2";
    public static double CAM1_X_OFFSET_TO_FRONT = -(0.236 + RobotConstants.BUMPER_WIDTH); //cam mounted 12.5" back from front bumper
    public static double CAM1_X_OFFSET_TO_CENTER = CAM1_X_OFFSET_TO_FRONT + RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE;
    public static double CAM1_Y_OFFSET_TO_CENTER = 0; //-0.034; //-0.013;
    public static Translation3d CAM1_POSITION_OFFSET = new Translation3d(CAM1_X_OFFSET_TO_CENTER, CAM1_Y_OFFSET_TO_CENTER,0.0); // is cam mounted at center? how far back from front of bumper?
    public static Rotation3d CAM1_ANGLE_OFFSET = new Rotation3d(0,0,0); // is cam mounted facing forward, upright? 

    //Camera 2
    public static final String CAM2_NAME = "Arducam OV9281 USB Camera"; 
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
