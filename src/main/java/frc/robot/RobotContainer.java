// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Ports;
import frc.robot.commands.Autos;
import frc.robot.commands.basic.*;
import frc.robot.subsystems.*;
import frc.robot.utils.TunerConstants;
import frc.robot.utils.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final Vision vision;

  private static final CommandXboxController driverController = new CommandXboxController(Ports.DRIVER_CONTROLLER);
  private static final CommandXboxController operatorController = new CommandXboxController(Ports.OPERATOR_CONTROLLER);
  private static final Drivetrain drivetrain = Drivetrain.getInstance();
  private static final Telemetry logger = new Telemetry(TunerConstants.MaxSpeed);


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
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.


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

    Drivetrain.getInstance().setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          TunerConstants.drive.withVelocityX(-driverController.getLeftY() * TunerConstants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-driverController.getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-driverController.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driverController.a().whileTrue(drivetrain.applyRequest(() -> TunerConstants.brake));
    driverController.b().whileTrue(drivetrain.applyRequest(() ->
        TunerConstants.point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
    ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);


    //---------- INTAKE ----------//
    new Trigger(Intake.getInstance()::isFuelJam).onTrue(new EatFuel());
    driverController.a().whileTrue(new EatFuel());




    //---------- HOPPER/LOADER ----------//



    //---------- SCORER ----------//



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // return new Command() {
    //   @Override
    //   public boolean isFinished() {
    //     return true;
    //   }
    // };
    // // An example command will be run in autonomous
    // return Autos.exampleAuto();

    // FROM TUNER: Simple drive forward auton
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain.applyRequest(() ->
            TunerConstants.drive.withVelocityX(0.5)
                .withVelocityY(0)
                .withRotationalRate(0)
        )
        .withTimeout(5.0),
        // Finally idle for the rest of auton
        drivetrain.applyRequest(() -> idle)
    );



  }
}
