package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Angler extends SubsystemBase {

  // ANGLER FIELDS

  private SparkMax angleMotor;
  private RelativeEncoder anglerEncoder; // Angle sensor UP/DOWN () == Relative position 
  public double anglerPosition = 0;
  private DigitalInput limitSwitch;
  private final SparkClosedLoopController pidController;
  private String side;


  // ANGLER CONSTRUCTOR
  public Angler(String side, int motorPort, int limitPort) {
    this.side = side;
    angleMotor = new SparkMax(motorPort, MotorType.kBrushless);
    anglerEncoder = angleMotor.getEncoder();
    limitSwitch = new DigitalInput(limitPort);    //REV throughbore connected to Angler Sparkmax
    pidController = angleMotor.getClosedLoopController();

    // PID gains for Angler
    SparkMaxConfig config = new SparkMaxConfig();

    config
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(30);

    config.encoder
      .positionConversionFactor( ((0.9/15.6)*360)/100) //0.9 and 15.6 from onshape
      .velocityConversionFactor(1);

    config.softLimit
    .forwardSoftLimitEnabled(true)
    .forwardSoftLimit(AnglerConstants.MIN_POSITION)
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(AnglerConstants.MAX_POSITION);
/* 
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.05)
      .i(0)
      .d(0.0005);

    // FeedForward for Angler
    config.closedLoop.feedForward
    .kG(0.00017)
    .kS(0.05);    // helps overcome initial friction
    
    // MAXMotion Velocity
    config.closedLoop.maxMotion
      .maxAcceleration(10000)      // RPM per second ramp up
      .allowedProfileError(50);    // Tolerance in RPM
 */
    // Apply the configuration to the motor
    angleMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
  }


  // BASIC ANGLER METHODS
  public void aimup(){
    angleMotor.set(1);
  }
  public void aimdown(){
    angleMotor.set(-1);
  }

  public void aimAngler(double direction){
    angleMotor.set(-direction);
  }

  public double getPosition(){  // get the angle of the angler
        return anglerEncoder.getPosition(); 
    }


  // BASIC ANGLER COMMANDS

  // In-line Command to move the hood down, producing a higher launch angle (closer shots)
  public Command angleUpCommand() {
    return run(
      () -> {
        aimup();
      });
  }

  // In-line Command to rotate the pitch angle of the hood based on provided speed
  public Command aimAnglerCommand(DoubleSupplier speedSupplier) {
    return run(
      () -> {
        aimAngler(speedSupplier.getAsDouble());
      });
  }

  // In-line Command to angle the angler up to a specific angle - in degrees
  public Command aimAnglerToSetPointCommand( DoubleSupplier anglerPosition) {
    return run(() -> {
      pidController.setSetpoint(anglerPosition.getAsDouble(), SparkMax.ControlType.kPosition);
    });
  }

  // In-line Command to angle the angler up to a specific angle - in degrees
  public Command aimAnglerToSetPointCommand( double anglerPosition) {
    return run(() -> {
      pidController.setSetpoint(anglerPosition, SparkMax.ControlType.kPosition);
    });
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(side + " anglerAngle", getPosition());
    if(!limitSwitch.get()){
      anglerEncoder.setPosition(0);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
