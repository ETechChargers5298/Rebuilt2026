// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Scorer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private static Scorer instance;
  TalonFX turretMotor;
  TalonFX angleMotor;
  TalonFX launcherMotor;
  public  double distanceFromHub = 0;
  public double angleToHub = 0;
  public double turretMotorSpeed = 0;
  public double turretAngle = 0;
  public double angleAngler = 0;
  // public double 

public static Scorer getInstance(){
  if (instance == null)
  instance = new Scorer();
  return instance;
}

  public Scorer() {

    turretMotor = new TalonFX(Ports.TURRET_MOTOR_PORT);
    angleMotor = new TalonFX(Ports.ANGLE_MOTOR_PORT);
    launcherMotor = new TalonFX(Ports.FLYWHEEL_MOTOR_PORT);
    
  
  }
  public void aimRight(){
    turretMotor.set(1.0);
  }

  public void aimLeft(){
    turretMotor.set(-1.0);
  }
  public void aimup(){
    angleMotor.set(1);
  }
  public void aimdown(){
    angleMotor.set(-1);
  }
  

  public double getLauncherSpeed(){
    
     StatusSignal<AngularVelocity> velocitySignal = launcherMotor.getVelocity();
    return velocitySignal.getValueAsDouble();

    // if (angleToHub == 0){
      
    //   turretX = 45;
    //   turretY = 45;
    // }
  }
  
  // public void getLancherSpeed(){
  //   if (distanceFromHub == 0){
  //   turretMotorSpeed = 1;
  //   }
    
  // }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
