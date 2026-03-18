package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.FeedbackSensor;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import com.revrobotics.spark.config.*;


public class Flywheel extends SubsystemBase {
  
  // FLYWHEEL FIELDS
  private SparkMax flywheelMotor;
  private final SparkClosedLoopController pidController;
  private RelativeEncoder flywheelEncoder; // Flywheel speed sensor (in sparkmax)
  public double setSpeed;
  private String side = "";

  // FLYWHEEL CONSTRUCTOR
  public Flywheel(String side, int motorPort) {
    
    flywheelMotor = new SparkMax(motorPort, MotorType.kBrushless);
    pidController = flywheelMotor.getClosedLoopController();
    flywheelEncoder = flywheelMotor.getEncoder();  //Built in encoder
    this.side = side;

    
    // PID gains for Flywheel
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kCoast);
/*
    config.encoder
      .positionConversionFactor(1)
      .velocityConversionFactor(1);

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0005)
      .i(0)
      .d(0);

    // FeedForward for Flywheel
    config.closedLoop.feedForward
    .kV(0.00017) // the main speed constant
    .kS(0.05);    // helps overcome initial friction
    
    // MAXMotion Velocity
    config.closedLoop.maxMotion
      .maxAcceleration(10000)      // RPM per second ramp up
      .allowedProfileError(50);    // Tolerance in RPM
   */

    // Apply the configuration to the motor
    flywheelMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  

  }


  // BASIC FLYWHEEL METHODS
  public void revFlywheel(){
    setSpeed = -1.0;
    flywheelMotor.set(setSpeed);
  }

  public void stopFlywheel(){
    flywheelMotor.set(0.0);
  }

  public double getFlywheelRpm(){
    //  StatusSignal<AngularVelocity> velocitySignal = launcherMotor.getVelocity();
    // return velocitySignal.getValueAsDouble();
    return flywheelEncoder.getVelocity();
  }

  public void speedUp(double speed){
    flywheelMotor.set(speed);
  }


  // BASIC FLYWHEEL COMMANDS

  public Command revFlywheelCommand() {
    return run(
      () -> {
        revFlywheel();
      });
  }

  public Command stopFlywheelCommand() {
    return run(
      () -> {
        stopFlywheel();
      });
    }
  
  public Command flyWheelCommand(DoubleSupplier speed){
    return run(
      () -> {
        speedUp(speed.getAsDouble());
      });
  }

  public void setReferenceRPM(double targetRPM) {
      pidController.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
  }

  // In-line Command to spin the flywheel up to a specific revolutions per minute
  public Command spinFlywheelToSetPointCommand( DoubleSupplier rpmSupplier) {
    return run(() -> this.setReferenceRPM(rpmSupplier.getAsDouble()));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(side + "flyWheelSpeed", getFlywheelRpm());
    SmartDashboard.putNumber(side + "Flywheel setSpeed", setSpeed);
    
  }
}
