package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants.*;


public class Turret extends SubsystemBase {

  // TURRET FIELDS
  private final TalonFX turretMotor;
  public double turretMotorSpeed = 0;
  public double turretAngle = 0;
  private String side;

  // Define the request once as a field to save memory
  private final MotionMagicVoltage expoRequest = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> positionSignal;

  // TURRET CONSTRUCTOR
  public Turret(String side, int motorPort) {

    turretMotor = new TalonFX(motorPort);

    this.side = side;

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    //Motion Magic PID gains (TUNE IT)
    config.Slot0.kP = TurretConstants.KP;
    config.Slot0.kI = TurretConstants.KI;
    config.Slot0.kD = TurretConstants.KD;
    config.Slot0.kV = TurretConstants.KV;
    config.Slot0.kS = TurretConstants.KS;

    //Motion Magic profile constraints
    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.ACCELERATION;
    config.MotionMagic.MotionMagicJerk = TurretConstants.JERK;

    // Configure Soft Limits directly on the Kraken hardware! (motor will stop even if code crashes)       
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ((TurretConstants.MAX_ANGLE + TurretConstants.EXTRA_DEGREES) / 360.0) * TurretConstants.GEAR_RATIO; // In Rotations
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ((TurretConstants.MIN_ANGLE - TurretConstants.EXTRA_DEGREES) / 360.0) * TurretConstants.GEAR_RATIO; // In Rotations
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretMotor.getConfigurator().apply(config);

    //Get angle
    positionSignal = turretMotor.getPosition();
    positionSignal.setUpdateFrequency(100);

    // Zero the motor on boot (Assumes turret is centered)      
    zeroTurretAngle();

  }
  
    
  // BASIC TURRET METHODS
  public void aimRight(){
    turretMotor.set(TurretConstants.AIM_SPEED);
  }

  public void aimLeft(){
    turretMotor.set(-TurretConstants.AIM_SPEED);
  }

  public double getTurretAngle(){ 
    return (positionSignal.refresh().getValueAsDouble() / TurretConstants.GEAR_RATIO) * 360;
  }

  public double getTurretRotation(){ 
    return (positionSignal.refresh().getValueAsDouble());
  }

  private double degreesToMotorRotations(double degrees){
    return (degrees / 360.0) * TurretConstants.GEAR_RATIO;
  }

  // Zero the angle of the turret (should be facing towards back of robot)
  public void zeroTurretAngle(){
    turretMotor.setPosition(0);
  }      


  // TURRET COMMANDS

  public void aimTurret(double speed){
    turretMotor.set(-speed);
  }

  // In-line Command to rotate the turret based on provided speed
  public Command moveTurretCommand(DoubleSupplier speedSupplier) {
    return run(
      () -> {
        aimTurret(speedSupplier.getAsDouble()/4);
      });
  }

  
  // Sets the turret to a specific angle using Motion Magic
  // targetTurretAngle Angle in degrees (will be modulus to -180 to 180)
  public void setTurretAngle(double targetTurretAngle) {
      
      // Normalize the angle so the turret takes the shortest path
      double relativeSetpoint = MathUtil.inputModulus(targetTurretAngle, -90, 90);
      
      // Convert degrees to Kraken rotations
      double targetRotations = degreesToMotorRotations(relativeSetpoint);
      
      // Apply the Motion Magic request to the hardware
      turretMotor.setControl(expoRequest.withPosition(targetRotations));
  }

  // In-line Command to rotate the turret to a specific angle - in degrees
  public Command aimTurretToSetPointCommand( Supplier<Double> turretAngleSupplier) {
    return run(() -> {
      this.setTurretAngle(-turretAngleSupplier.get());
    });
  }

  public Command aimTurretToSetPointCommand(double turretAngle){
    return run(() -> {
      this.setTurretAngle(turretAngle);
    });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(side + " turretAngle", getTurretAngle());
    SmartDashboard.putNumber(side + " turretRotations:", getTurretRotation());
    
  }
  


}
