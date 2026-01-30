package frc.robot.subsystems;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;


// IMPORT STATEMENTS
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.Utils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import frc.robot.Ports;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.utils.*;
import frc.robot.FieldConstants;



public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {

  // DRIVETRAIN FIELDS
  private static Drivetrain instance = null;

  // ETECH FIELDS
  private double xSpeed = 0.0;
  private double ySpeed = 0.0;
  private double rotSpeed = 0.0;
  public boolean fieldCentric = true;
  public boolean allianceCentric = true;
  private final Field2d field;

  // CTRE SIM FIELDS
  private static final double kSimLoopPeriod = 0.004; // 4 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // CTRE ALLIANCE FIELDS
  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  // CTRE PATHPLANNER FIELDS
  /** Swerve request to apply during robot-centric path following */
  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
  
  // CTRE SYSID FIELDS
  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();





  // OLD FIELDS
  // private final List<SwerveModule> modulesList;  
  // private static final SwerveDriveKinematics driveKinematics;
  // private final SwerveDrivePoseEstimator poseEstimator;




  //--------------------- CONSTRUCTORS -----------------------------------------------//


  /** Drivetrain Constructor #1
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * This constructs the underlying hardware devices, so users should not construct
   * the devices themselves. If they need the devices, they can access them through
   * getters in the classes.
   *
   * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
   * @param modules               Constants for each specific module
   */
  private Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
  ) {
    super(drivetrainConstants, modules);
    this.field = new Field2d();
    swerveInit();
  }

  /** Drivetrain Constructor #2
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    private Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        this.field = new Field2d();
        swerveInit();
    }

  /** Drivetrain Constructor #3
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    private Drivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        this.field = new Field2d();
        swerveInit();
    }

  public void swerveInit(){

    this.fieldCentric = true;
    this.allianceCentric = true;    
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    // this.poseEstimator =  new SwerveDrivePoseEstimator(
    //   SwerveConstantsOld.DRIVE_KINEMATICS,
    //   getRobotHeading(),
    //   getSwerveModulePos(),
    //   FieldConstants.getRobotPoseInitialFMS().toPose2d(), // Starting pose based on FMS Alliance + Driver Station
    //   stateStdDevs,
    //   visionStdDevs);

    // autoConfig();
    // sysIdConfig();

    if (Utils.isSimulation()) {
      startSimThread();
    }

    configureAutoBuilder();

  }


  // Drivetrain Singleton - ensures only 1 instance of Drivetrain is constructed
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeftModule,
        TunerConstants.FrontRightModule,
        TunerConstants.BackLeftModule,
        TunerConstants.BackRightModule
        // TunerConstants.modules
      );
    }
    return instance;
  }



  //------------------------------ PATHPLANNER Methods ------------------------------------//
  private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }


  //------------------------------ CTRE Methods -------------------------------------------------//
  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> request) {
    return run(() -> this.setControl(request.get()));
  }



  //--------------------------- OLD DT Methods -------------------------------------------//
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
    //SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
    
    //cleans up any weird speeds that may be too high after kinematics equation
   // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstantsOld.TOP_SPEED);

    // setting the state for each module as an array
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].setDesiredState(swerveModuleStates[i]);
    // }
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    // frontL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    // frontR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    // backL.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    // backR.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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


    
  //---------------[OLD] SWERVEMODULE HELPER METHODS --------------//

  // public ChassisSpeeds getSpeeds() {
  //   return driveKinematics.toChassisSpeeds(getModuleStates());
  // }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstantsOld.TOP_SPEED);
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].setDesiredState(desiredStates[i]);
    // }
  }

  // method to return all the positions of the 4 modules
  public SwerveModulePosition[] getSwerveModulePos() {  
    SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];
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
    // poseEstimator.resetPosition(getRobotHeading(), getSwerveModulePos(), newPose);
  }

  public void updatePoseFromOdometry() {
    // poseEstimator.update(getRobotHeading(), getSwerveModulePos());
  }

  public Field2d getField() {
    return field;
  }


  /**
   * Returns the currently-estimated pose of the robot relative to the FIELD
   * @return The pose.
   */
  public Pose2d getPose() {
    // return poseEstimator.getEstimatedPosition();
    return new Pose2d();
  }

  // public double getFieldAngleDegrees(){
  //   return getPose().getRotation().getDegrees();
  // }

  // public double getFieldAngleRadians(){
  //   return getPose().getRotation().getRadians();
  // }

  


  public void updateModuleTelemetry() {
    // for(int i = 0; i < modules.length; i++) {
    //   modules[i].updateTelemetry();
    // }
  }



   // Updates pose estimate based on 2 cameras (used by Vision class)
  // public void updateEstimates(EstimatedRobotPose estimatedPose1, Matrix<N3, N1> standardDev1, EstimatedRobotPose estimatedPose2, Matrix<N3, N1> standardDev2) {
    
  //   Pose3d estimate1 = new Pose3d();
  //   Pose3d estimate2 = new Pose3d();
    
  //   estimate1 = estimatedPose1.estimatedPose;
  //   estimate2 = estimatedPose2.estimatedPose;

  //   poseEstimator.addVisionMeasurement(
  //     estimate1.toPose2d(),
  //     estimatedPose1.timestampSeconds,
  //     standardDev1
  //   );

  //   poseEstimator.addVisionMeasurement(
  //     estimate2.toPose2d(),
  //     estimatedPose2.timestampSeconds,
  //     standardDev2
  //   );

  //   field.getObject("Cam 1 Est Pose").setPose(estimate1.toPose2d());
  //   field.getObject("Cam 2 Est Pose").setPose(estimate2.toPose2d());

  // }






  // ----------------------------- SIM METHODS -----------------------------------------//
  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
        final double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }



  // ----------------------------- SYSID ----------------------------------------------------//
  
  /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,        // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null,        // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)),
          null,
          this
      )
  );

  /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
  private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
      new SysIdRoutine.Config(
          null,        // Use default ramp rate (1 V/s)
          Volts.of(7), // Use dynamic voltage of 7 V
          null,        // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          volts -> setControl(m_steerCharacterization.withVolts(volts)),
          null,
          this
      )
  );

  /*
    * SysId routine for characterizing rotation.
    * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
    * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    */
  private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
      new SysIdRoutine.Config(
          /* This is in radians per second², but SysId only supports "volts per second" */
          Volts.of(Math.PI / 6).per(Second),
          /* This is in radians per second, but SysId only supports "volts" */
          Volts.of(Math.PI),
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
      ),
      new SysIdRoutine.Mechanism(
          output -> {
              /* output is actually radians per second, but SysId only supports "volts" */
              setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
              /* also log the requested output for SysId */
              SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
          },
          null,
          this
      )
  );

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return m_sysIdRoutineToApply.dynamic(direction);
  }




  //----------------------- PATHPLANNER OLD ---------------------------------------------------------//

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



  // --------------------------  ALLIANCE COLOR ------------------------------------------//

  public void checkAllianceColor(){
    /*
      * Periodically try to apply the operator perspective.
      * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
      * This allows us to correct the perspective in case the robot code restarts mid-match.
      * Otherwise, only check and apply the operator perspective if the DS is disabled.
      * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
      */
    if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        });
    }



  }


  //--------------------------- VISION ------------------------------------------------------//

  // Pose Estimator controlled in CTRE's SwerveDrivetrain super class!


  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   */
  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
      super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
   * while still accounting for measurement noise.
   * Note that the vision measurement standard deviations passed into this method
   * will continue to apply to future measurements until a subsequent call to
   * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
   * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
   * @param timestampSeconds The timestamp of the vision measurement in seconds.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
   *     in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs
  ) {
      super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }



  /**
   * Return the pose at a given timestamp, if the buffer is not empty.
   *
   * @param timestampSeconds The timestamp of the pose in seconds.
   * @return The pose at the given timestamp (or Optional.empty() if the buffer is empty).
   */
  @Override
  public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
      return super.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
  }




  //--------------------------- PERIODIC -------------------------------------------------//

  
  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    updatePoseFromOdometry();
    updateModuleTelemetry();
    move(this.xSpeed, this.ySpeed, this.rotSpeed, this.fieldCentric, this.allianceCentric);
    
    // SmartDashboard.putNumber("NavX Compass Heading", navX.getCompassHeading());

    SmartDashboard.putNumber("Robot Angle Degrees", getRobotAngleDegrees());
    SmartDashboard.putNumber("Robot Angle Radians", getRobotAngleRadians());
    // SmartDashboard.putNumber("Field Angle Degrees", getFieldAngleDegrees());
    // SmartDashboard.putNumber("Field Angle Radians", getFieldAngleRadians());
    

    // SmartDashboard.putNumber("PoseX", getPose().getX());
    // SmartDashboard.putNumber("PoseY", getPose().getY());
    // SmartDashboard.putNumber("PoseAngle", getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("xspeed", xSpeed);
    SmartDashboard.putNumber("yspeed", ySpeed);
    SmartDashboard.putNumber("rotspeed", rotSpeed);

    // field.setRobotPose(getPose());
    SmartDashboard.putData("PoseEstimator Field", field);
    SmartDashboard.putBoolean("fieldCentric", fieldCentric);
    // SmartDashboard.putNumber("FL distanceMeters", frontL.getPosition().distanceMeters);

    checkAllianceColor();

  }

    


}  // end Drivetrain class
