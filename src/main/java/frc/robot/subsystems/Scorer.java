// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Locale.LanguageRange;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.encoder.DetachedEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;

public class Scorer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // FIELDS
  private static Scorer instance;
  private SparkMax turretMotor;
  private SparkMax angleMotor;
  private SparkMax flywheelMotor;

  private RelativeEncoder anglerEncoder; // Angle sensor UP/DOWN () == Relative position 
  private RelativeEncoder turretEncoder; // turret sensor LEFT/RIGHT() == relative position with throughbore
  private RelativeEncoder flywheelEncoder; // Flywheel speed sensor (in sparkmax)


  public double distanceFromHub = 0;
  public double angleToHub = 0;
  public double turretMotorSpeed = 0;
  public double turretAngle = 0;
  public double angleAngler = 0;



// //funcitons
// Angling (sparkmax)
// Flywheel to lanch (sparkmax)
// Turret aiming LEFTRIGHT (sparkmax)



// ball sensor in the launcher ()




  public static Scorer getInstance(){
  if (instance == null)
    instance = new Scorer();
    return instance;
  }

  public Scorer() {

    turretMotor = new SparkMax(Ports.TURRET_MOTOR_PORT, MotorType.kBrushless);
    angleMotor = new SparkMax(Ports.ANGLE_MOTOR_PORT, MotorType.kBrushless);
    flywheelMotor = new SparkMax(Ports.FLYWHEEL_MOTOR_PORT, MotorType.kBrushless);
    turretEncoder = turretMotor.getAlternateEncoder();    //REV throughbore connected to Turret Sparkmax
    anglerEncoder = angleMotor.getAlternateEncoder();   //REV throughbore connected to Angler Sparkmax
    flywheelEncoder = flywheelMotor.getEncoder();  //Built in encoder
  }


  // BASIC FUNCTIONALITY
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

  public void revFlywheel(){
    flywheelMotor.set(1.0);
  }

  public void aimTurret(double direction){
    turretMotor.set(direction);
  }

  public void aimAngler(double direction){
    angleMotor.set(direction);
  }

  // SENSOR ACCESSOR METHODS
  public double getLauncherSpeed(){
    
    //  StatusSignal<AngularVelocity> velocitySignal = launcherMotor.getVelocity();
    // return velocitySignal.getValueAsDouble();

    return flywheelEncoder.getVelocity();
  


    // if (angleToHub == 0){
      
    //   turretX = 45;
    //   turretY = 45;
    // }
  }
  

  public double getAnglerAngle(){

    return anglerEncoder.getPosition(); 

  }


  public double getTurretAngle(){
    
    return turretEncoder.getPosition();
  }



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command revFlywheelCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          revFlywheel();
        });
  }

  public Command angleUpCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          aimup();
        });
  }



  public Command aimTurretCommand(DoubleSupplier speedSupplier) {
    return runOnce(
        () -> {
          aimTurret(speedSupplier.getAsDouble());
        });
  }

  public Command aimAnglerCommand(DoubleSupplier speedSupplier) {
    return runOnce(
        () -> {
          aimAngler(speedSupplier.getAsDouble());
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

    SmartDashboard.putNumber("flyWheelSpeed", getLauncherSpeed());
    SmartDashboard.putNumber("anglerAngle", getAnglerAngle());
    SmartDashboard.putNumber("turretAngle", getTurretAngle());


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
