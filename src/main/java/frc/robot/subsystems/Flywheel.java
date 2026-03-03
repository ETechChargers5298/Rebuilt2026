package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Flywheel extends SubsystemBase {
  
  // FLYWHEEL FIELDS
  private SparkMax flywheelMotor;
  private RelativeEncoder flywheelEncoder; // Flywheel speed sensor (in sparkmax)
  public double setSpeed;


  // FLYWHEEL CONSTRUCTOR
  public Flywheel(String side, int motorPort) {
    flywheelMotor = new SparkMax(motorPort, MotorType.kBrushless);
    flywheelEncoder = flywheelMotor.getEncoder();  //Built in encoder
  }

  // BASIC FLYWHEEL METHODS
  public void revFlywheel(){
    setSpeed = -1.0;
    flywheelMotor.set(setSpeed);
  }

  public void stopFlywheel(){
    flywheelMotor.set(0.0);
  }

  public double getLauncherSpeed(){
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

  // In-line Command to spin the flywheel up to a specific revolutions per minute
  public Command spinFlywheelToSetPointCommand( Supplier<Double> flywheelRpm) {


    return null;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("flyWheelSpeed", getLauncherSpeed());
    SmartDashboard.putNumber("Flywheel setSpeed", setSpeed);
    
  }
}
