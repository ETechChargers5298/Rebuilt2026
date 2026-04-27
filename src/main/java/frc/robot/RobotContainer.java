package frc.robot;

import frc.robot.commands.autos.*;
// import frc.robot.commands.PivotIntake;
import frc.robot.subsystems.*;
import frc.robot.utils.TunerConstants;
import frc.robot.utils.Telemetry;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  // SUBSYSTEM CONSTRUCTIONS
  public final ScorerLeft scorerLeft = ScorerLeft.getInstance();
  public final ScorerRight scorerRight = ScorerRight.getInstance();
  public final Loader loader = Loader.getInstance();
  public final IntakePivot intake = IntakePivot.getInstance();
  private final Vision vision = Vision.getInstance();    //VISION! (Comment IN to use vision)
  public final Music music = Music.getInstance();
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
  private final CommandXboxController testingController = new CommandXboxController(Ports.TESTING_CONTROLLER);
  private final Drivetrain drivetrain = Drivetrain.getInstance();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  //PATHPLANNER FIELDS
  private final SendableChooser<Command> autoChooser;
  // Command c1 = AutoBuilder.followPath(PathPlannerPath.fromPathFile("forward1m.json"));

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Start a webserver to host the Elastic layout file
    // On any computer connected to the robot, open Elastic, go to the File menu, and select Load Layout From Robot (or press Ctrl + D)
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    // Constructs autoChooser WITHOUT automatically populating from PathPlanner
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("NULL", null);

    // Constructs an auto chooser that populates all commands from PathPlanner, and includesa default command
    // autoChooser = AutoBuilder.buildAutoChooser("Forward 1 meter");
    
    // Auto Options
    autoChooser.addOption("AUTO: Straight to Depot (StartLeftBump)", new StraightToDepotAuto());
    autoChooser.addOption("AUTO: Trench Left Auto", new TrenchLeftAuto());
    autoChooser.addOption("AUTO: Trench Right Auto", new TrenchRightAuto());

    
    // Add Auto Chooser to Elastic
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Warmup PathPlanner to avoid Java pauses
    FollowPathCommand.warmupCommand().schedule();

    // Setup joystick buttons
    configureBindings();

    //Setup drive motors for music playing
    music.addInstrument(drivetrain.getModule(2).getSteerMotor(), 1);
    music.addInstrument(drivetrain.getModule(2).getDriveMotor(), 2);

    music.addInstrument(drivetrain.getModule(1).getSteerMotor(), 3);
    music.addInstrument(drivetrain.getModule(1).getDriveMotor(), 4);

    music.addInstrument(drivetrain.getModule(3).getSteerMotor(), 5);
    music.addInstrument(drivetrain.getModule(3).getDriveMotor(), 6);

    music.addInstrument(drivetrain.getModule(4).getSteerMotor(), 7);
    music.addInstrument(drivetrain.getModule(4).getDriveMotor(), 8);

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
    // new Trigger(loader::isLoaderActive).whileTrue(drivetrain.applyRequest(() -> brake));

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

    // SWERVE SysId ROUTINES (TESTING - BACK/START + X/Y)
    // Note: Each routine should be run exactly once in a single log.
    testingController.back().and(testingController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    testingController.back().and(testingController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    testingController.start().and(testingController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    testingController.start().and(testingController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // TRIGGER SIGNAL LOGGING ( TESTING - LB/RB)
    testingController.leftBumper().onTrue(Commands.runOnce(SignalLogger:: start));
    testingController.rightBumper().onTrue(Commands.runOnce(SignalLogger:: stop));

    // FIELD-CENTRIC HEADING RESET (DRIVER - X)
    // driverController.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    
    // Ensure that the telemetry is updated from our drivetrain's movements
    drivetrain.registerTelemetry(logger::telemeterize);


    //---------- INTAKE JOYSTICK CONTROLLER BINDINGS----------//

    // INTAKEROLLERS STOPS by default
    IntakeRollers.getInstance().setDefaultCommand(  IntakeRollers.getInstance().stopEatingCommand()  );

    // EAT FUEL (DRIVER - RT)
    driverController.rightTrigger().whileTrue( IntakeRollers.getInstance().eatFuelCommand() );

    // SPIT FUEL (DRIVER - LT)
    driverController.leftTrigger().whileTrue(  IntakeRollers.getInstance().spitFuelCommand()   );


    // INTAKE PIVOT maintains position by default
    IntakePivot.getInstance().setDefaultCommand(  IntakePivot.getInstance().stopPivotingCommand()  );

    // INTAKE PIVOT EXTEND OUT (OPERATOR - B) (DRIVER - RB)
    operatorController.b().whileTrue( IntakePivot.getInstance().extendCommand()  );
    driverController.rightBumper().whileTrue( IntakePivot.getInstance().extendCommand()  );
    
    // INTAKE PIVOT RETRACT UP (OPERATOR - X) (DRIVER - LB)
    operatorController.x().whileTrue( IntakePivot.getInstance().retractCommand());
    driverController.leftBumper().whileTrue( IntakePivot.getInstance().retractCommand());

    // INTAKE PIVOT 6-7 (DRIVER - X)
    driverController.x().whileTrue(IntakePivot.getInstance().sixSevenCommand());
     
    
    //---------- LOADER JOYSTICK CONTROLLER BINDINGS ----------//
   
    // STOPS LOADER by default
    Loader.getInstance().setDefaultCommand(Loader.getInstance().stopLoadCommand());

    // LOAD IN (OPERATOR - RT)
    operatorController.rightTrigger().whileTrue( Loader.getInstance().loadInCommand());
    testingController.rightTrigger().whileTrue( Loader.getInstance().loadInCommand());
    // LOAD OUT (OPERATOR - ???)
    //operatorController.???().whileTrue(loader.unloadCommand());



    //---------- FLYWHEEL JOYSTICK CONTROLLER  BINDINGS----------//

    // STOP FLYWHEEL by default
    scorerLeft.flywheel.setDefaultCommand(scorerLeft.flywheel.stopFlywheelCommand()  ); //comment out for FlyWheel testing
    scorerRight.flywheel.setDefaultCommand(scorerRight.flywheel.stopFlywheelCommand()  ); //comment out for FlyWheel testing

    // REV FLYWHEEL MANUALLY (OPERATOR - RB)
    operatorController.rightBumper().whileTrue(scorerLeft.flywheel.revFlywheelCommand());
    operatorController.rightBumper().whileTrue(scorerRight.flywheel.revFlywheelCommand());

    // operatorController.rightBumper().whileTrue(scorerLeft.flywheel.flyWheelCommand(() -> 0.5));
    // operatorController.rightBumper().whileTrue(scorerRight.flywheel.flyWheelCommand(() -> 0.5))

    operatorController.leftBumper().whileTrue(Loader.getInstance().unloadCommand());


    // INCREMENT/DECREMENT FLYWHEEL SPEEDS (TESTING - POV LEFT/RIGHT)
    testingController.povRight().onTrue(new InstantCommand(() -> {
      double setSpeed = scorerLeft.flywheel.getSetSpeed();
      scorerLeft.flywheel.setSetSpeed(setSpeed + 0.01);
    }));
    testingController.povLeft().onTrue(new InstantCommand(() -> {
      double setSpeed = scorerLeft.flywheel.getSetSpeed();
      scorerLeft.flywheel.setSetSpeed(setSpeed - 0.01);
    }));
    
    
    testingController.povRight().onTrue(new InstantCommand(() -> {
      double setSpeed = scorerRight.flywheel.getSetSpeed();
      scorerRight.flywheel.setSetSpeed(setSpeed + 0.01);
    }));
    testingController.povLeft().onTrue(new InstantCommand(() -> {
      double setSpeed = scorerRight.flywheel.getSetSpeed();
      scorerRight.flywheel.setSetSpeed(setSpeed - 0.01);
    }));

    //---------- ANGLER JOYSTICK CONTROLLER BINDINGS ----------//

    // AIM ANGLER MANUALLY (OPERATOR - LY AXIS)
    scorerLeft.angler.setDefaultCommand(
      scorerLeft.angler.aimAnglerCommand( () -> MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) )
    );
    scorerRight.angler.setDefaultCommand(
      scorerRight.angler.aimAnglerCommand( () -> MathUtil.applyDeadband(operatorController.getLeftY(), 0.1) )
    );

    //AIM ANGLER TO SETPOINT 0 (OPERATOR - L3) (TESTING - L3)
    operatorController.leftStick().whileTrue(scorerLeft.angler.aimAnglerCommand(() -> 0));
    testingController.leftStick().whileTrue(scorerLeft.angler.aimAnglerCommand(() -> 0));
    operatorController.leftStick().whileTrue(scorerRight.angler.aimAnglerCommand(() -> 0));
    testingController.leftStick().whileTrue(scorerRight.angler.aimAnglerCommand(() -> 0));

    // INCREMENT/DECREMENT ANGLER ANGLES (TESTING - POV UP/DOWN)
    testingController.povUp().onTrue(new InstantCommand(() -> {
      double setAngle = scorerLeft.angler.getPosition();
      scorerLeft.angler.setSetpoint(setAngle + 0.01);
    }));
    testingController.povDown().onTrue(new InstantCommand(() -> {
      double setAngle = scorerLeft.angler.getPosition();
      scorerLeft.angler.setSetpoint(setAngle - 0.01);
    }));
    testingController.povUp().onTrue(new InstantCommand(() -> {
      double setAngle = scorerRight.angler.getPosition();
      scorerRight.angler.setSetpoint(setAngle + 0.01);
    }));
    testingController.povDown().onTrue(new InstantCommand(() -> {
      double setAngle = scorerRight.angler.getPosition();
      scorerRight.angler.setSetpoint(setAngle - 0.01);
    }));

    //---------- TURRET JOYSTICK CONTROLLER BINDINGS ----------//

    // AIM TURRET MANUALLY (OPERATOR - RX AXIS)
    scorerLeft.turret.setDefaultCommand(
      scorerLeft.turret.moveTurretCommand( () -> MathUtil.applyDeadband(operatorController.getRightX(), 0.1) )
    );
    scorerRight.turret.setDefaultCommand(
      scorerRight.turret.moveTurretCommand( () -> MathUtil.applyDeadband(operatorController.getRightX(), 0.1) )
    );

    // AIM TURRET TO SETPOINT 0 (OPERATOR - R3)
    operatorController.rightStick().whileTrue(scorerLeft.turret.aimTurretToSetPointCommand(0));
    operatorController.rightStick().whileTrue(scorerRight.turret.aimTurretToSetPointCommand(0));

    // CHANGE THE AIMING TARGET (OPERATOR - POV UP/LEFT/RIGHT)
    operatorController.povUp().onTrue(new InstantCommand( () -> scorerLeft.setTargetToHub()));
    operatorController.povLeft().onTrue(new InstantCommand( () -> scorerLeft.setTargetToHerdDepot()));
    operatorController.povRight().onTrue(new InstantCommand( () -> scorerLeft.setTargetToHerdOutpost()));

    operatorController.povUp().onTrue(new InstantCommand( () -> scorerRight.setTargetToHub()));
    operatorController.povLeft().onTrue(new InstantCommand( () -> scorerRight.setTargetToHerdDepot()));
    operatorController.povRight().onTrue(new InstantCommand( () -> scorerRight.setTargetToHerdOutpost()));

    //---------- SCORER CONTROLLER BINDINGS ----------//

    // AIM SCORER TO TARGET (OPERATOR - LT)
    operatorController.leftTrigger().whileTrue(scorerLeft.AimToTarget());
    operatorController.leftTrigger().whileTrue(scorerRight.AimToTarget());

    // BONUS BUMPS (OPERATOR - A & Y)
    operatorController.y().onTrue( ScorerLeft.getInstance().bonusUpCommand()  );
    operatorController.a().onTrue( ScorerLeft.getInstance().bonusDownCommand()  );

    operatorController.y().whileTrue( ScorerRight.getInstance().bonusUpCommand()  );
    operatorController.a().whileTrue( ScorerRight.getInstance().bonusDownCommand()  );

    // Test Controller Commands
    // testingController.leftStick();
    driverController.leftStick().and(driverController.rightStick()).whileTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */    
   public Command getAutonomousCommand() {

      //FROM CTRE PATHPLANNERpublic Command getAutonomousCommand() {
      // This method loads the auto when it is called, however, it is recommended
      // to first load your paths/autos when code starts, then return the
      // pre-loaded auto/path
      // return new PathPlannerAuto("Example Auto");
      return autoChooser.getSelected();

    }

  }
