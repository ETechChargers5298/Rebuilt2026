// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Scorer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Launcher extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final Scorer scorer;

  private double kP = 0, kI = 0, kD = 0;
  private PIDController launchPid = new PIDController(kP, kI, kD);
  private double setPoint = 0;


  public Launcher() {
    scorer = Scorer.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scorer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    launchPid.setTolerance(5,10);

    launchPid.setSetpoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // scorer.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
