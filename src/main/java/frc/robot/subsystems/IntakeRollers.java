package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class IntakeRollers extends SubsystemBase {

  // INTAKE FIELDS
  private static IntakeRollers instance;
  private SparkMax eatMotor;

  // INTAKE CONSTRUCTOR
  private IntakeRollers() {
    eatMotor = new SparkMax(Ports.EAT_MOTOR_PORT,MotorType.kBrushless);
  }

  // INTAKE SINGLETON - ensures only 1 instance of Intake is constructed
  public static IntakeRollers getInstance() {
    if (instance == null) {
      instance = new IntakeRollers();
    }  
    return instance;
  }

  // BASIC INTAKE METHODS

  public void eat() {
    eatMotor.set(-IntakeConstants.EAT_SPEED);
  }
  
  public void spit()
  {
    eatMotor.set(IntakeConstants.EAT_SPEED);
  }

  public void stopEating()
  {
    eatMotor.set(0);
  }



  // BASIC INTAKE COMMANDS

  // In-line Command to eat fuel off the ground into the hopper
  public Command eatFuelCommand() {
    return run(
        () -> {
          eat();
        });
  }

  // In-line Command to spit fuel from the hopper back onto the ground
  public Command spitFuelCommand() {
    return run(
        () -> {
          spit();
        });
  }

  // In-line Command to stop moving the intake rollers
  public Command stopEatingCommand(){
    return run(
      () -> {
        stopEating();
      });
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
