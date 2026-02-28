// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Locale.LanguageRange;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import frc.robot.subsystems.Drivetrain;

public class Turret extends SubsystemBase {

  // TURRET FIELDS
  private static Turret instance;
  //private TalonFX turretMotor;
  // private RelativeEncoder turretEncoder; // turret sensor LEFT/RIGHT() == relative position with throughbore

  
  public double distanceFromHub = 0;
  public double angleToHub = 0;
  public double turretMotorSpeed = 0;
  public double turretAngle = 0;
  public double angleAngler = 0;

  // TURRET CONSTRUCTOR
  //private Turret() {
    //turretMotor = new SparkMax(Ports.TURRET_MOTOR_PORT, MotorType.kBrushless);
  private final TalonFX turretMotor = new TalonFX(Ports.TURRET_MOTOR_PORT);

    //motor gear ratio
    private final double GEAR_RATIO = 10.0;
    // turretEncoder = turretMotor.getAlternateEncoder();    //REV throughbore connected to Turret Sparkmax
    public Turret() {

      TalonFXConfiguration config = new TalonFXConfiguration();
    // Configure Soft Limits directly on the Kraken hardware!
    // This is safer because the motor stops itself even if the code crashes.
      
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (375.0 / 360.0) * GEAR_RATIO; // In Rotations
        
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (-5.0 / 360.0) * GEAR_RATIO;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
    turretMotor.getConfigurator().apply(config);
        
    // Zero the motor on boot (Assumes turret is centered)      
    turretMotor.setPosition(0);
  }
  
  // TURRET SINGLETON
  public static Turret getInstance(){
    if (instance == null)
      instance = new Turret();
      return instance;
  }
    
  // BASIC TURRET METHODS
  public void aimRight(){
    turretMotor.set(1.0);
  }

  public void aimLeft(){
    turretMotor.set(-1.0);
  }

  public void aimTurret(double direction){
    turretMotor.set(direction);
  }



//
  public double getAngleToHubFromRobotPerspective(){
      double hubX = 182.11; //From field drawings, not sure which is X or Y
      double hubY = 317.69 / 2; //From field drawings
      double robotX = Drivetrain.getInstance().getRobotX();
      double robotY = Drivetrain.getInstance().getRobotY();

    return Math.toDegrees(Math.atan2(hubY - robotY, hubX - robotX));
  }

  public double getAngleToHubFromTurretPerspective(){
    return 180 - getAngleToHubFromRobotPerspective();
  }

  //  public double getTurretAngle(){ 
  //   return turretEncoder.getPosition();
  // }

  // TURRET COMMANDS

  // In-line Command to rotate the turret based on provided speed
  public Command moveTurretCommand(DoubleSupplier speedSupplier) {
    return run(
      () -> {
        aimTurret(speedSupplier.getAsDouble()/4);
      });
  }


  // In-line Command to rotate the turret to a specific angle - in degrees
  public Command aimTurretToSetPointCommand( Supplier<Double> turretAngle) {
  

    return null;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("turretAngle", getTurretAngle());
  }
}
