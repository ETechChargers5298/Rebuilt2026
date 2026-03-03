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
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.Drivetrain;

public class Turret extends SubsystemBase {

  // TURRET FIELDS
  private final TalonFX turretMotor;
  // private RelativeEncoder turretEncoder; // turret sensor LEFT/RIGHT() == relative position with throughbore

  public double turretMotorSpeed = 0;
  public double turretAngle = 0;
  public double angleAngler = 0;
  private final double GEAR_RATIO = 100.0 / 10.0;      // gear ratio of turret (Big gear of 100: Small gear of 10)
  private final double EXTRA_DEGREES = 5.0;      // additional degrees beyond 360 the turret should rotate in each direction
  
  
  // TURRET CONSTRUCTOR
  public Turret(String side, int motorPort) {

    turretMotor = new TalonFX(motorPort);

    // turretEncoder = turretMotor.getAlternateEncoder();    //REV throughbore connected to Turret Sparkmax_

    TalonFXConfiguration config = new TalonFXConfiguration();

    // Configure Soft Limits directly on the Kraken hardware! (motor will stop even if code crashes)       
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ((360.0 + EXTRA_DEGREES) / 360.0) * GEAR_RATIO; // In Rotations
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (0 -EXTRA_DEGREES / 360.0) * GEAR_RATIO; // In Rotations
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotor.getConfigurator().apply(config);
        
    // Zero the motor on boot (Assumes turret is centered)      
    zeroTurretAngle();
  }
  
    
  // BASIC TURRET METHODS
  public void aimRight(){
    turretMotor.set(1.0);
  }

  public void aimLeft(){
    turretMotor.set(-1.0);
  }

  public void aimTurret(double speed){
    turretMotor.set(speed);
  }

  public double getTurretAngle(){ 
    return turretMotor.getPosition().getValueAsDouble();
  }

  // Zero the angle of the turret (should be facing towards back of robot)
  public void zeroTurretAngle(){
    turretMotor.setPosition(0);
  }      


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
    SmartDashboard.putNumber("turretAngle", getTurretAngle());
  }
}
