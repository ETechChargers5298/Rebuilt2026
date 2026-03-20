package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.AnglerConstants;
import frc.robot.Constants.IntakeConstants;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class IntakePivot extends SubsystemBase {

  // INTAKE FIELDS
  private static IntakePivot instance;
  private SparkMax pivotMotorRight; // All extendMotor related things are currntly placeholder
  private SparkMax pivotMotorLeft; // All extendMotor related things are currntly placeholder
  private RelativeEncoder pivotEncoder;
  // private final SparkClosedLoopController pidController;


  // INTAKE PIVOT CONSTRUCTOR
  private IntakePivot() {
    pivotMotorRight = new SparkMax(Ports.EXTEND_MOTOR_RIGHT_PORT,MotorType.kBrushless);
    pivotMotorLeft = new SparkMax(Ports.EXTEND_MOTOR_LEFT_PORT,MotorType.kBrushless);
    pivotEncoder = pivotMotorLeft.getEncoder();
    // pidController = pivotMotorLeft.getClosedLoopController();
    

    // Motor Configs
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);
    
    // // config.encoder
    //   .positionConversionFactor(IntakeConstants.GEAR_RATIO)
    //   .velocityConversionFactor(1);
    
    // config.softLimit
    //   .forwardSoftLimitEnabled(true)
    //   .forwardSoftLimit(IntakeConstants.UP_ANGLE)
    //   .reverseSoftLimitEnabled(true)
    //   .reverseSoftLimit(IntakeConstants.DOWN_ANGLE);

    // config.closedLoop
    //     .pid(0.1, 0, 0.01)
    //     .outputRange(-0.5, 0.5); //can throttle the voltage if necessary


    // Have Right motor follow the Left motor to be in sync
    followerConfig
      .follow(pivotMotorLeft, true)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40);

    // Apply configs to motors
    // pivotMotorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // pivotMotorRight.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  // INTAKE SINGLETON - ensures only 1 instance of Intake is constructed
  public static IntakePivot getInstance() {
    if (instance == null) {
      instance = new IntakePivot();
    }  
    return instance;
  }

  // BASIC INTAKE METHODS

  public void generalExtend(double speed)
  {
    pivotMotorRight.set(speed);
    pivotMotorLeft.set(-speed);
  }

  public void extend()
  {
    pivotMotorRight.set(IntakeConstants.EXTEND_SPEED);
    pivotMotorLeft.set(IntakeConstants.EXTEND_SPEED);
  }

  public void retract()
  {
    pivotMotorRight.set(-IntakeConstants.RETRACT_SPEED);
    pivotMotorLeft.set(-IntakeConstants.RETRACT_SPEED);
  }

  public void stopExtending(){
    
    pivotMotorRight.set(0);
    pivotMotorLeft.set(0);
  }

  public double getPivotAngle()
  {
    return pivotEncoder.getPosition();
  }

  // Check if the fuel is jammed
  public boolean isFuelJam(){
    return false;
  }


  // BASIC INTAKE COMMANDS

  // In-line Command to eat fuel off the ground into the hopper
  public Command extendCommand() {
    return run(
        () -> {
          extend();
        }).finallyDo(
          () -> {
              stopExtending();
          }
        );
  }

  // In-line Command to spit fuel from the hopper back onto the ground
  public Command retractCommand() {
    return run(
        () -> {
          retract();
        }).finallyDo(
          () -> {
              stopExtending();
          }
        );
  }

  // In-line Command to stop moving the intake rollers
  public Command stopPivotingCommand(){
    return run(
      () -> {
        stopExtending();
      });
  }


  // In-line Command to pivot Intake to a specific angle - in degrees
  // public Command pivotToSetPointCommand(DoubleSupplier targetAngle) {
  //   return run(() -> {
  //     pidController.setSetpoint(targetAngle.getAsDouble(), ControlType.kPosition);
  //   });
  // }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Pivot Angle", getPivotAngle());
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
