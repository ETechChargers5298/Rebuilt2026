package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;


public class Conveyor extends SubsystemBase {

  // CONVEYOR FIELDS
  private static Conveyor instance;
  private SparkMax conveyorMotor;


  // CONVEYOR CONSTRUCTOR
  private Conveyor() 
  {
    // conveyorMotor = new SparkMax(Ports.CONVEYOR_MOTOR_PORT, MotorType.kBrushless);
  }

  // CONVEYOR SINGLETON
  public static Conveyor getInstance()
  {
    if(instance == null)
    {
      instance = new Conveyor();
    }
    return instance;
  }

  // CONVEYOR METHODS
  public void conveyIn()
  {
    conveyorMotor.set(1.0);
  }

  public void conveyOut()
  {
    conveyorMotor.set(-1.0);
  }

  public void stopConvey()
  {
    conveyorMotor.set(0);
  }

  public void generalConvey(double speed)
  {
    conveyorMotor.set(speed);
  }

  // CONVEYOR BASIC COMMANDS
  public Command conveyInCommand() {
    return run(
        () -> {
          conveyIn();
        });
  }

  public Command conveyOutCommand() {
    return run(
        () -> {
          conveyOut();
        });
  }

  public Command stopConveyCommand() {
    // Inline construction of command goes here.
    return run(
        () -> {
          stopConvey();
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
