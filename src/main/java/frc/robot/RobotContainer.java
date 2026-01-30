// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.basic.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private static final CommandXboxController driverController = new CommandXboxController(Ports.DRIVER_CONTROLLER);
  private static final CommandXboxController operatorController = new CommandXboxController(Ports.OPERATOR_CONTROLLER);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    //vision = Vision.getInstance();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Link for joystick doc: https://docs.google.com/presentation/d/1cis5OrQfkU9m38LwgAMIfmPpJAZxnIC-KnAzi0JsRao/edit#slide=id.g18d2b75b637cb431_3

    //---------- DRIVETRAIN ----------//
    
    //Driver - LX & LY joysticks for Translation, RX joystick for Strafing, A to reset Robot NavX Heading
    // Drivetrain.getInstance().setDefaultCommand(
    
    //   new SwerveDrive(
    //     () -> driverController.getRawAxis(1),
    //     () -> -driverController.getRawAxis(0),
    //     () -> -driverController.getRawAxis(4), //negative joystick values make a positive CCW turn
    //     () -> driverController.getAButton()
    //   )
    
    // );

    //---------- INTAKE ----------//
    // new Trigger(Intake.getInstance()::isFuelJam).onTrue(new EatFuel());
    // new JoystickButton(driverController,Button.kA.value).whileTrue(new EatFuel());


    //---------- HOPPER/LOADER ----------//



    //---------- SCORER ----------//
    operatorController.rightBumper().onTrue(Scorer.getInstance().revFlywheelCommand());


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return true;
      }
    };
    // // An example command will be run in autonomous
    // return Autos.exampleAuto();
  }
}
