// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class MoveIntake extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Intake intake;

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private PIDController pid;

  private double setPoint = 0;

  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveIntake(double point) {
    
    intake = Intake.getInstance();
    pid = new PIDController(kP, kI, kD);
    setPoint = point;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setTolerance(5,10);
    pid.setSetpoint(setPoint);  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intake.generalExtend(pid.calculate(intake.getExtendAngle()));

  }

  // Called once the command ends (when the pid is at the setpoint)
  @Override
  public void end(boolean interrupted) {
    pid.close();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
