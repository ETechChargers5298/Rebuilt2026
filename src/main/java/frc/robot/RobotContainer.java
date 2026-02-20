// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Ports;
import frc.robot.commands.Autos;
import frc.robot.commands.PivotIntake;
// import frc.robot.commands.PivotIntake;
import frc.robot.commands.basic.*;
import frc.robot.subsystems.*;
import frc.robot.utils.TunerConstants;
import frc.robot.utils.Telemetry;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.net.WebServer; // For the WebServer class
import edu.wpi.first.wpilibj.Filesystem; // For the Filesystem class

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  //VISION! (Comment IN to use vision)
   private final Vision vision = Vision.getInstance();

  // CTRE SWERVE FIELDS
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


  // ROBOTCONTAINER FIELDS
  private final CommandXboxController driverController = new CommandXboxController(Ports.DRIVER_CONTROLLER);
  private final CommandXboxController operatorController = new CommandXboxController(Ports.OPERATOR_CONTROLLER);
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  //PATHPLANNER FIELDS
  // private final SendableChooser<Command> autoChooser;
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Start a webserver to host the Elastic layout file
    // On any computer connected to the robot, open Elastic, go to the File menu, and select Load Layout From Robot (or press Ctrl + D)
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // autoChooser = AutoBuilder.buildAutoChooser("Tests");
    // SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
    // Warmup PathPlanner to avoid Java pauses
    // FollowPathCommand.warmupCommand().schedule();
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

    // Link for map of Joystick Controllers: https://docs.google.com/presentation/d/1W7NYwlgP1rfN15ja3PuY3GGtcSFDY97u9_2YbxYBe2A/edit?slide=id.g18d2b75b637cb431_3#slide=id.g18d2b75b637cb431_3


    //---------- DRIVETRAIN JOYSTICK CONTROLLER BINDINGS----------//
    
    // BASIC SWERVE DRIVING WITH 3 AXES (DRIVER - LY= forward, LX = strafe, RX = turn)
    // Note: WPILib convention of X is , Y is left
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() ->
          TunerConstants.drive.withVelocityX(-driverController.getLeftY() * TunerConstants.MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(-driverController.getLeftX() * TunerConstants.MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-driverController.getRightX() * TunerConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );

    // SWERVE IDLE WHILE DISABLED
    // Note: This ensures the configured neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    // SWERVE BRAKE (DRIVER - A)
    driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

    // SWERVE TURN TO DIRECTION OF STRAFE (DRIVER - B button)
    driverController.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
    ));

    // SWERVE FORWARD HALF-SPEED (DRIVER - DPAD_UP)
    driverController.povUp().whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0.5).withVelocityY(0))
    );

    // SWERVE BACKWARD HALF-SPEED (DRIVER - DPAD_DOWN)
    driverController.povDown().whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(-0.5).withVelocityY(0))
    );


    // SWERVE SysId ROUTINES (DRIVER - BACK/START + X/Y)
    // Note: Each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // FIELD-CENTRIC HEADING RESET (DRIVER - LB)
    driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // Ensure that the telemetry is updated from our drivetrain's movements
    drivetrain.registerTelemetry(logger::telemeterize);

    //OLD CTRE DRIVE COMMAND
    //Driver - LX & LY joysticks for Translation, RX joystick for Strafing, A to reset Robot NavX Heading
    // Drivetrain.getInstance().setDefaultCommand(
    
    //   new SwerveDrive(
    //     () -> driverController.getRawAxis(1),
    //     () -> -driverController.getRawAxis(0),
    //     () -> -driverController.getRawAxis(4), //negative joystick values make a positive CCW turn
    //     () -> driverController.getAButton()
    //   )
    
    // );



    //---------- INTAKE JOYSTICK CONTROLLER BINDINGS----------//

    // INTAKE STOPS by default
    Intake.getInstance().setDefaultCommand(  Intake.getInstance().stopEatingCommand()  );

    // EAT FUEL (OPERATOR - LB)
   // operatorController.leftBumper().whileTrue( Intake.getInstance().eatFuelCommand() );

    // SPIT FUEL (OPERATOR - LT)
    operatorController.leftTrigger().whileTrue(  Intake.getInstance().spitFuelCommand()   );


    // PIVOT maintains position by default

    // PIVOT INTAKE OUT (OPERATOR - B)
    operatorController.b().whileTrue(new PivotIntake(45));//extend intake temp point
    
    // RETRACT INTAKE BACK (OPERATOR - X)
    operatorController.x().whileTrue(new PivotIntake(0));//retract intake temp point
   



    //---------- CONVEYOR JOYSTICK CONTROLLER BINDINGS ----------//

    // STOPS CONVEYER by default
    Conveyor.getInstance().setDefaultCommand(Conveyor.getInstance().stopConveyCommand());

    // CONVEY IN (OPERATOR - A)
    operatorController.a().whileTrue( Conveyor.getInstance().conveyInCommand());
      
    // CONVEY OUT (OPERATOR - Y)
    operatorController.y().whileTrue( Conveyor.getInstance().conveyOutCommand());
      

    //---------- LOADER JOYSTICK CONTROLLER BINDINGS ----------//

    // LOAD FUEL TO LAUNCHER (OPERATOR - RT)

    
    // STOPS LOADER by default
    Loader.getInstance().setDefaultCommand(Loader.getInstance().stopLoadCommand());

    // LOAD IN (OPERATOR - RT)
    operatorController.rightTrigger().whileTrue( Loader.getInstance().loadInCommand());
      
    // LOAD OUT (OPERATOR - ???)
    //operatorController.???().whileTrue(Loader.getInstance().unloadCommand());



    //---------- FLYWHEEL JOYSTICK CONTROLLER  BINDINGS----------//

    // STOP FLYWHEEL by default
    Flywheel.getInstance().setDefaultCommand(Flywheel.getInstance().revFlywheelCommand()  );

    // REV FLYWHEEL (OPERATOR - RB)
    operatorController.rightBumper().onTrue(new InstantCommand(() -> {

      Flywheel.getInstance().setSpeed += 0.01;
      Flywheel.getInstance().setSpeed = Math.min(Flywheel.getInstance().setSpeed, 0);

    }));
    

    
    operatorController.leftBumper().onTrue(new InstantCommand(() -> {

      Flywheel.getInstance().setSpeed -= 0.01;
      Flywheel.getInstance().setSpeed = Math.max(Flywheel.getInstance().setSpeed, -1);

    }));
    // Flywheel.getInstance().setDefaultCommand(
    //   Flywheel.getInstance().flyWheelCommand( () -> MathUtil.applyDeadband(operatorController.getLeftX(), 0.1) )
    // );

    //---------- ANGLER JOYSTICK CONTROLLER BINDINGS ----------//

    // AIM ANGLER (OPERATOR - RY AXIS)
    Angler.getInstance().setDefaultCommand(
      Angler.getInstance().aimAnglerCommand( () -> MathUtil.applyDeadband(operatorController.getRightY(), 0.1) )
    );
    // ANGLE UP (OPERATOR - X) - will not stop moving without defaul Angler command
    // operatorController.x().whileTrue(Scorer.getInstance().angleUpCommand());
    

    //---------- TURRET JOYSTICK CONTROLLER BINDINGS ----------//

    // AIM TURRET (OPERATOR - LX AXIS)
  //   Turret.getInstance().setDefaultCommand(
  //     Turret.getInstance().aimTurretCommand( () -> MathUtil.applyDeadband(operatorController.getLeftX(), 0.1) )
  //   );
   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //FROM CTRE PATHPLANNER
    //return autoChooser.getSelected();
    
    // return new Command() {
    //   @Override
    //   public boolean isFinished() {
    //     return true;
    //   }
    // };
    // // An example command will be run in autonomous
    // return Autos.exampleAuto();

    // FROM CTRE TUNER: Simple drive forward auton
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
