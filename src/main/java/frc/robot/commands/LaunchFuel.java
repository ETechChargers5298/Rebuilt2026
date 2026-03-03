// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ScorerLeft;
import frc.robot.subsystems.ScorerRight;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LaunchFuel extends Command {

  private final ScorerLeft scorerLeft;
  private final ScorerRight scorerRight;

  private double kP = 0, kI = 0, kD = 0;
  private PIDController launchPid = new PIDController(kP, kI, kD);
  private double setPoint = 0;


  public LaunchFuel() {
    scorerLeft = ScorerLeft.getInstance();
    scorerRight = ScorerRight.getInstance();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scorerLeft.flywheel, scorerRight.flywheel);
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
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
