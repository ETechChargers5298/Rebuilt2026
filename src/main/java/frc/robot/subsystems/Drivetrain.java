package frc.robot.subsystems;


// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;

import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.FieldConstants;
import frc.robot.utils.SwerveModule;


public class Drivetrain extends SubsystemBase {

  private static Drivetrain instance;

  private final List<SwerveModule> modulesList;
  private final SwerveDriveKinematics driveKinematics;

  private final SwerveModule frontL = new SwerveModule();
  private final SwerveModule frontR = new SwerveModule();
  private final SwerveModule backL = new SwerveModule();
  private final SwerveModule backR = new SwerveModule();

//   public AHRS navX;   // The gyro sensor

  // Slew rate filter variables for controlling lateral acceleration
  // private double m_currentRotSpeed = 0.0;
  // private double m_currentTranslationDir = 0.0;
  // private double m_currentTranslationMag = 0.0;
  // private SlewRateLimiter m_magLimiter = new SlewRateLimiter(SwerveConstants.kMagnitudeSlewRate);
  // private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(SwerveConstants.kRotationalSlewRate);
  // private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  //Fields that control 3 dimensions of drive motion
  private double xSpeed = 0.0;
  private double ySpeed = 0.0;
  private double rotSpeed = 0.0;
  public boolean fieldCentric = true;
  public boolean allianceCentric = true;

  private final SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field;


  // SysID Commands
  public Command transQ1;
  public Command transQ2;
  public Command transD1;
  public Command transD2;
  public Command rotQ1;
  public Command rotQ2;
  public Command rotD1;
  public Command rotD2;


  /** Drivetrain Constructor */
  private Drivetrain() {

    // this.modules = new SwerveModule[4];
    // modules[0] = frontL;
    // modules[1] = frontR;
    // modules[2] = backL;
    // modules[3] = backR;

    modulesList = new ArrayList<SwerveModule>();
    modulesList.add(frontL);
    modulesList.add(frontR);
    modulesList.add(backL);
    modulesList.add(backR);
    
    //assign the NavX to be our sensor for rotation
    //*****no worky figure out why*******
    //this.navX = new AHRS(SPI.Port.kMXP);
    // this.navX = new AHRS(NavXComType.kMXP_SPI);

    this.driveKinematics = SwerveConstants.DRIVE_KINEMATICS;

    this.field = new Field2d();
    this.fieldCentric = true;
    this.allianceCentric = true;
    
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    this.poseEstimator =  new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      getRobotHeading(),
      getSwerveModulePos(),
      FieldConstants.getRobotPoseInitialFMS().toPose2d(), // Starting pose based on FMS Alliance + Driver Station
      stateStdDevs,
      visionStdDevs);

    // autoConfig();
    // sysIdConfig();

  }

  //PathPlanner Drive Controller
//   public final PPHolonomicDriveController pathFollowerConfig = new PPHolonomicDriveController(
//     new PIDConstants(SwerveAutoConstants.TRANSLATE_P, SwerveAutoConstants.TRANSLATE_I, SwerveAutoConstants.TRANSLATE_D), // Translation constants 
//     new PIDConstants(SwerveAutoConstants.TURN_P, SwerveAutoConstants.TURN_I, SwerveAutoConstants.TURN_D) // Rotation constants 
//   );

  // Configure AutoBuilder for PathPlanner
//   private void autoConfig(){

//     AutoBuilder.configure(
//       this::getPose, 
//       this::resetPose, 
//       this::getSpeeds, 
//       this::driveRobotRelative, 
//       pathFollowerConfig,
//       new RobotConfig(
//         RobotConstants.MASS, 
//         RobotConstants.MOI, 
//         new ModuleConfig(
//           SwerveModuleConstants.WHEEL_DIAMETER_METERS/2, 
//           SwerveConstants.TOP_SPEED, 
//           SwerveModuleConstants.WHEEL_COEFFICIENT_OF_FRICTION, 
//           DCMotor.getNEO(1).withReduction(SwerveModuleConstants.DRIVE_GEAR_REDUCTION), 
//           SwerveModuleConstants.kDrivingMotorCurrentLimit, 
//           1), 
//         Constants.SwerveConstants.DRIVE_KINEMATICS.getModules()
//       ),
//       () -> {
//           // Boolean supplier that controls when the path will be mirrored for the red alliance
//           // This will flip the path being followed to the red side of the field.
//           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//           var alliance = DriverStation.getAlliance();
//           if (alliance.isPresent()) {
//               return alliance.get() == DriverStation.Alliance.Red;
//           }
//           return false;
//       },
//       this
//     );
   
//   }

//   private void sysIdConfig(){
    
//     // Create the SysId routines
//     var translationSysIdRoutine = new SysIdRoutine(
//       new SysIdRoutine.Config(),
//       new SysIdRoutine.Mechanism(
//         volts ->
//         modulesList.forEach(
//             m -> m.updateInputs(Rotation2d.fromRadians(0), volts.in(Volts))),
//         // (voltage) -> this.runVolts(voltage.in(Volts)),
//         null, // No log consumer, since data is recorded by URCL
//         this
//       )
//     );
//     var rotationalSysIdRoutine = new SysIdRoutine(
//       new SysIdRoutine.Config(),
//       new SysIdRoutine.Mechanism(
//         volts -> {
//           this.frontL.updateInputs(
//             Rotation2d.fromRadians((3 * Math.PI / 4) + SwerveConstants.FL_ANGULAR_OFFSET), volts.in(Volts));
//         this.frontR.updateInputs(
//             Rotation2d.fromRadians((Math.PI / 4) + SwerveConstants.FR_ANGULAR_OFFSET), volts.in(Volts));
//         this.backL.updateInputs(
//             Rotation2d.fromRadians((-3 * Math.PI / 4) + SwerveConstants.BL_ANGULAR_OFFSET), volts.in(Volts));
//         this.backR.updateInputs(
//             Rotation2d.fromRadians((-Math.PI / 4) + SwerveConstants.BR_ANGULAR_OFFSET), volts.in(Volts));
//         },
//         // (voltage) -> this.runVolts(voltage.in(Volts)),
//         null, // No log consumer, since data is recorded by URCL
//         this
//       )
//     );

//     // SysID methods below return Command objects
//     transQ1 = translationSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
//     transQ2 = translationSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
//     transD1 = translationSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
//     transD2 = translationSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);

//     rotQ1 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
//     rotQ2 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
//     rotD1 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
//     rotD2 = rotationalSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);

//   }


  // Drivetrain Singleton - ensures only 1 instance of Drivetrain is constructed
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  // sets forward/backward motion of robot
  public void setXSpeed(double xSpeed){
    this.xSpeed = xSpeed;
  }

  // sets strafing right/left speed of robot
  public void setYSpeed(double ySpeed){
    this.ySpeed = ySpeed;
  }

  // sets rotation right/left speed of robot
  public void setRotSpeed(double rotSpeed){
    this.rotSpeed = rotSpeed;
  }

  // sets whether driving is fieldcentric or not
  public void setFieldCentric(boolean fieldCentric) {
    this.fieldCentric = fieldCentric;
  }  
  public boolean getFieldCentric() {
    return fieldCentric;
  }

  /**
   * Making a drive function to make the speed for drive a fraction of total
   * @author Aiden Sing
   * @param xSpeed speed of the robot front to back
   * @param ySpeed speed of robot left to right
   * @param rotSpeed speed of robot turning
   */
  public void setDrive(double xSpeed, double ySpeed, double rotSpeed) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
  }

  public void setDrive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldCentric, boolean allianceCentric) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.fieldCentric = fieldCentric;
    this.allianceCentric = allianceCentric;
  }

  public void stopDrive() {
    setDrive(0.0,0.0,0.0);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldcentric Whether the provided x and y speeds are relative to the field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void move(double xSpeed, double ySpeed, double rot, boolean fieldcentric, boolean allianceCentric) {

    double xSpeedCommanded = -xSpeed;
    double ySpeedCommanded = ySpeed;
    double rotSpeedCommanded = rot;

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded;
    double ySpeedDelivered = ySpeedCommanded;
    double rotSpeedDelivered = rotSpeedCommanded;

    //SwerveModuleState[] 
    // var swerveModuleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
    //     fieldcentric
    //         ? ChassisSpeeds.fromfieldcentricSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
    //         : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    //Store an array of speeds for each wheel. By default do robot centric speeds but if fieldCentric use fromFieldRelativeSpeeds
    ChassisSpeeds speeds = new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered);

    if (fieldCentric) {
      var rotation = getPose().getRotation();
      
      if(allianceCentric) {
      var allianceOptional = DriverStation.getAlliance();
      if (allianceOptional.isPresent() && allianceOptional.get() == DriverStation.Alliance.Red) {
        // Flip the rotation if our driverstation is red alliance so that driving is "driver centric"
        rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
      }
    }
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotSpeedDelivered, rotation);
    }

    //Store the states of each module
    SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
    
    //cleans up any weird speeds that may be too high after kinematics equation
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    // setting the state for each module as an array
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].setDesiredState(swerveModuleStates[i]);
    // }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // Helps AutoBuilder do stuff - ONLY USED BY PATH PLANNER
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {

    SmartDashboard.putNumber("PP Xspeed", robotRelativeSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("PP Yspeeds", robotRelativeSpeeds.vyMetersPerSecond);

    double speedFactor = 1;
    
    ChassisSpeeds speeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    //negative Y-values fix something
    
    this.move(-speeds.vxMetersPerSecond * speedFactor, speeds.vyMetersPerSecond * speedFactor, speeds.omegaRadiansPerSecond, false, false);


    //Store the states of each module
    // SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
    
    // //cleans up any weird speeds that may be too high after kinematics equation
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED);

    // // setting the state for each module as an array
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].setDesiredState(swerveModuleStates[i]);
    // }

  }


    
  //---------------SWERVEMODULE HELPER METHODS --------------//

  public ChassisSpeeds getSpeeds() {
    return driveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.TOP_SPEED);
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].setDesiredState(desiredStates[i]);
    // }
  }

  // method to return all the positions of the 4 modules
  public SwerveModulePosition[] getSwerveModulePos() {  
    SwerveModulePosition[] modulePosition = {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
    
    // for(int i = 0; i < modules.length; i++) {
    //   SwerveModulePosition currentPos = modules[i].getPosition();
    //   modulePosition[i] = new SwerveModulePosition(currentPos.distanceMeters, currentPos.angle); //negative on distance BAD!
    // }

    return modulePosition;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    // for (int i = 0; i < modules.length; i++) {
    //   states[i] = modules[i].getState();
    // }
    return states;
  }

  
  
  //---------------NAVX METHODS --------------//

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getRobotHeading() {
    //return Rotation2d.fromDegrees(navX.getAngle());
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
        // return navX.getRotation2d();
    return Rotation2d.fromDegrees(0.0);
  }

  public double getRobotAngleDegrees() {
    return getRobotHeading().getDegrees();
  }

 public double getRobotAngleRadians() {
    return getRobotHeading().getRadians();
  }

  // Resets the drive encoders to currently read a position of 0.
  public void resetEncoders() {
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].resetEncoders();
    // }
  }

  // Zeroes the heading of the robot, previously called resetIMU()
  public void zeroRobotHeading() {
    // navX.reset();
  }


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // return navX.getVelocityZ() * (SwerveConstants.TURN_INVERSION ? -1.0 : 1.0);
    return 0.0;
    //return m_gyro.getRate(IMUAxis.kZ) * (SwerveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public float getPitch() {
    // return navX.getPitch();
    return 0.0f;
  }

  public float getRoll() {
    // return navX.getRoll();
    return 0.0f;
  }



    //---------------POSE ESTIMATION METHODS --------------//
   /**
   * Resets the poseEstimator to the specified pose.
   * @param pose The pose to which to set the poseEstimator
   */
  public void resetPose(Pose2d newPose) {
    poseEstimator.resetPosition(getRobotHeading(), getSwerveModulePos(), newPose);
  }

  public void updatePoseFromOdometry() {
    poseEstimator.update(getRobotHeading(), getSwerveModulePos());
  }

  public Field2d getField() {
    return field;
  }

  public SwerveDriveKinematics getKinematics() {
    return driveKinematics;
  }

  /**
   * Returns the currently-estimated pose of the robot relative to the FIELD
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getFieldAngleDegrees(){
    return getPose().getRotation().getDegrees();
  }

  public double getFieldAngleRadians(){
    return getPose().getRotation().getRadians();
  }

  
  // /* See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  // public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
  //   poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  // }

  /* See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
    Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  public void updateModuleTelemetry() {
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].updateTelemetry();
    // }
  }



   // Updates pose estimate based on 2 cameras (used by Vision class)
  public void updateEstimates(EstimatedRobotPose estimatedPose1, Matrix<N3, N1> standardDev1, EstimatedRobotPose estimatedPose2, Matrix<N3, N1> standardDev2) {
    
    Pose3d estimate1 = new Pose3d();
    Pose3d estimate2 = new Pose3d();
    
    estimate1 = estimatedPose1.estimatedPose;
    estimate2 = estimatedPose2.estimatedPose;

    poseEstimator.addVisionMeasurement(
      estimate1.toPose2d(),
      estimatedPose1.timestampSeconds,
      standardDev1
    );

    poseEstimator.addVisionMeasurement(
      estimate2.toPose2d(),
      estimatedPose2.timestampSeconds,
      standardDev2
    );

    field.getObject("Cam 1 Est Pose").setPose(estimate1.toPose2d());
    field.getObject("Cam 2 Est Pose").setPose(estimate2.toPose2d());

  }





  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    updatePoseFromOdometry();
    updateModuleTelemetry();
    move(this.xSpeed, this.ySpeed, this.rotSpeed, this.fieldCentric, this.allianceCentric);
    
    // SmartDashboard.putNumber("NavX Compass Heading", navX.getCompassHeading());

    SmartDashboard.putNumber("Robot Angle Degrees", getRobotAngleDegrees());
    SmartDashboard.putNumber("Robot Angle Radians", getRobotAngleRadians());
    SmartDashboard.putNumber("Field Angle Degrees", getFieldAngleDegrees());
    SmartDashboard.putNumber("Field Angle Radians", getFieldAngleRadians());
    

    SmartDashboard.putNumber("PoseX", getPose().getX());
    SmartDashboard.putNumber("PoseY", getPose().getY());
    SmartDashboard.putNumber("PoseAngle", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rotspeed", rotSpeed);

    field.setRobotPose(getPose());
    SmartDashboard.putData("PoseEstimator Field", field);
    SmartDashboard.putBoolean("fieldCentric", fieldCentric);
    SmartDashboard.putNumber("FL distanceMeters", frontL.getPosition().distanceMeters);
  }
}