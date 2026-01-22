package frc.robot.utils;


// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModuleConstants;
// import frc.robot.utils.ModuleConfig;

public class SwerveModule {

    // private final SparkMax driveMotor;
    // private final SparkMax turnMotor;

    // private final RelativeEncoder driveEncoder;
    // private final SparkAbsoluteEncoder turnEncoder;
    
    // private final SparkClosedLoopController driveController;
    // private final SparkClosedLoopController turnController;

    // private final SparkMaxConfig driveMotorConfig;
    // private final SparkMaxConfig turnMotorConfig;

    // private final ClosedLoopConfig driveControllerConfig;
    // private final ClosedLoopConfig turnControllerConfig;

    private SwerveModuleState desiredState; 

    // public final ModuleConfig config;

//     private final VelocityVoltage velocityOut = new VelocityVoltage(0);
//     private final PositionVoltage rotationsIn = new PositionVoltage(0);
  /** Creates a new SwerveModule. */
  public SwerveModule() {
    // this.config = config;
/* 
    //This intitilizes the Drive and Turn motors.
    driveMotor = new SparkMax(config.DRIVE_PORT, MotorType.kBrushless);
    turnMotor = new SparkMax(config.TURN_PORT, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getAbsoluteEncoder();

    //PID Controllers 
    driveController = driveMotor.getClosedLoopController();
    turnController = turnMotor.getClosedLoopController();


    desiredState = new SwerveModuleState(0.0, new Rotation2d() );

    //Istablished the Motor configations with the new Spark Configs
    driveMotorConfig = new SparkMaxConfig();
    turnMotorConfig = new SparkMaxConfig();

    driveControllerConfig = new ClosedLoopConfig();
    turnControllerConfig = new ClosedLoopConfig();

    

    // driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    // driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // turnMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    // turnMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    
    
    // //References the established constants and and tells the Drive SparkMax what to set its PID to
    // driveControllerConfig.p(ModuleConstants.kTurningP);
    // driveControllerConfig.i(ModuleConstants.kTurningI);
    // driveControllerConfig.d(ModuleConstants.kTurningD);
    // driveControllerConfig.velocityFF(ModuleConstants.kDrivingFF);
    // driveControllerConfig.outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

    // //References the established constants and and tells the Turn SparkMax what to set its PID to
    // turnControllerConfig.p(ModuleConstants.kTurningP);
    // turnControllerConfig.i(ModuleConstants.kTurningI);
    // turnControllerConfig.d(ModuleConstants.kTurningD);
    // turnControllerConfig.velocityFF(ModuleConstants.kTurningFF);
    // turnControllerConfig.outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    // driveMotorConfig.idleMode(ModuleConstants.kDrivingMotorIdleMode);
    // turnMotorConfig.idleMode(ModuleConstants.kTurningMotorIdleMode);

    // driveMotorConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    // turnMotorConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // driveControllerConfig.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // turnControllerConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);


    // //flips the drive motor 
    driveMotorConfig.inverted(config.DRIVE_INVERSION);
    //driveMotorConfig.encoder.inverted(true);

    double drivingFactor = SwerveModuleConstants.WHEEL_DIAMETER_METERS * Math.PI
            / SwerveModuleConstants.DRIVE_GEAR_REDUCTION;
    double turningFactor = 2 * Math.PI;
    double drivingVelocityFeedForward = 1 / SwerveModuleConstants.kDriveWheelFreeSpeedRps;

    driveMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    driveMotorConfig.encoder
            .positionConversionFactor(drivingFactor) // meters
            .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      
    driveMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(0.04, 0, 0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1, 1);

    turnMotorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
    turnMotorConfig.absoluteEncoder
            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of the steering motor in the MAXSwerve Module.
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
    turnMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);

    //Resets motors back to original
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnMotor.configure(turnMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // give the motors current controller configuration
    driveMotorConfig.apply(driveControllerConfig);
    turnMotorConfig.apply(turnControllerConfig);
    
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveEncoder.setPosition(0);

    */
  }

  public SwerveModuleState getState(){

    return new SwerveModuleState(0.0, new Rotation2d(0.0));
    // return new SwerveModuleState(driveEncoder.getVelocity(),
    //  new Rotation2d(turnEncoder.getPosition() - config.ANGULAR_OFFSET)); 
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(0.0, new Rotation2d(0.0));
    // return new SwerveModulePosition(driveEncoder.getPosition(),
    //  new Rotation2d(turnEncoder.getPosition() - config.ANGULAR_OFFSET));
  }

  public double getTurnRadians(){
    return getPosition().angle.getRadians();
  }

  public void setDesiredState(SwerveModuleState desiredState){
    
    // SwerveModuleState correctedDesiredState = new SwerveModuleState();
    // correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    // correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(config.ANGULAR_OFFSET));


    // //Maybe come back and fix, potentially not what we want
    // correctedDesiredState.optimize(new Rotation2d(turnEncoder.getPosition()));

    // driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    // turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    // this.desiredState = desiredState;
     
  }

  
  public void updateInputs(Rotation2d angle, double voltage) {
    desiredState.angle = angle;
    setDriveVoltage(voltage);
    setTurnSetpoint(angle);
  }

public void setDriveVoltage(double voltage) {
//   driveMotor.setVoltage(voltage);
}

public void setTurnVoltage(double voltage) {
//   turnMotor.setVoltage(voltage);
}

public void setTurnSetpoint(Rotation2d angle) {
  //turnMotor.setControl(rotationsIn.withPosition(angle.getRotations()).withSlot(0));
//   turnController.setReference(angle.getRadians(), ControlType.kPosition);
}

   public void resetEncoders() {
    // driveEncoder.setPosition(0);
  }

  public void updateTelemetry() {
    // SmartDashboard.putNumber(config.NAME + " Angle Degrees", getPosition().angle.getDegrees());
    // SmartDashboard.putNumber(config.NAME + " Angle Radians", getTurnRadians());
    // SmartDashboard.putNumber(config.NAME + " Drive Position", getPosition().distanceMeters);
    // SmartDashboard.putNumber(config.NAME + "Drive Motor Voltage", driveMotor.getBusVoltage());
  }


}