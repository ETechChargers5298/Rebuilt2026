package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.FieldConstants;


public class Scorer {

    // SCORER FIELDS
    public String side;
    public Turret turret;
    public Angler angler;
    public Flywheel flywheel;
    public double distanceFromHub = 0;
    public double angleToHub = 0;
    private double xOffset;
    private double yOffset;
    private double hubX = 0;
    private double hubY = FieldConstants.FIELD_CENTER_Y; //meters
    double robotX = Drivetrain.getInstance().getRobotX();
    double robotY = Drivetrain.getInstance().getRobotY();
    double robotAngle = Drivetrain.getInstance().getRobotAngleDegrees();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    static Alliance alliance = null;

    // Tolerances for the Scorer
    private final double TURRET_TOLERANCE_DEG = 1.5;
    private final double FLYWHEEL_TOLERANCE_RPM = 100.0;
    private final double ANGLER_TOLERANCE_DEG = 0.5;
    
    

    // SCORER CONSTRUCTOR
    public Scorer(String side, double xOffset, double yOffset){
        this.side = side;
        
        if(side.equals("LEFT")){
            this.turret = TurretLeft.getInstance();
            this.angler = AnglerLeft.getInstance();
            this.flywheel = FlywheelLeft.getInstance();

        } else if(side.equals("RIGHT")){
            this.turret = TurretRight.getInstance();
            this.angler = AnglerRight.getInstance();
            this.flywheel = FlywheelRight.getInstance();

        } else {
            System.out.println("Error constructing Scorer");
        }


         if (allianceOptional.isPresent()) {
            alliance = allianceOptional.get();
        }  


        if(alliance == DriverStation.Alliance.Blue){
            hubX = FieldConstants.BLUE_HUB_CENTER_X;
        } else {
            hubX = FieldConstants.RED_HUB_CENTER_X;
        }

    }



    // SCORER METHODS

    // Get the turret's X position in field coordinates (rotated by robot heading)
    private double getTurretX(){
        double angleRad = Math.toRadians(robotAngle);
        return robotX + xOffset * Math.cos(angleRad) - yOffset * Math.sin(angleRad);
    }

    // Get the turret's Y position in field coordinates (rotated by robot heading)
    private double getTurretY(){
        double angleRad = Math.toRadians(robotAngle);
        return robotY + xOffset * Math.sin(angleRad) + yOffset * Math.cos(angleRad);
    }

    // Distance from TurretCenter -->  Alliance Hub Center
    public double getDistanceToHub(){
        double tx = getTurretX();
        double ty = getTurretY();
        return Math.sqrt(Math.pow(hubX - tx, 2) + Math.pow(hubY - ty, 2));
    }

    // Angle from HubCenter --> TurretCenter --> Field X-axis 0
    public double getAngleToHubFromFieldPerspective(){
        return Math.toDegrees(Math.atan2(hubY - getTurretY(), hubX - getTurretX()));
    }
 
    // Angle from HubCenter --> TurretCenter --> RobotFront X-axis (front/intake)
    public double getAngleToHubFromRobotPerspective(){
        return getAngleToHubFromFieldPerspective() - robotAngle;
    }

    // Angle from HubCenter - ScorerCenter - Turret X-axis
    // if you are at the turret's center (facing backward/starting angle), to which angle is the hub?
    public double getAngleToHubFromTurretPerspective(){
        return (180.0 - getAngleToHubFromRobotPerspective()) - 360;
    }


    // check if the Scorer mechs are ready for a launch
    public boolean isReadyToScore() {
        
        double dist = getDistanceToHub();
        var params = getIdealShot(dist);
        
        // Check if Turret is aimed towards Hub
        boolean turretReady = Math.abs(turret.getTurretAngle() - getAngleToHubFromTurretPerspective()) < TURRET_TOLERANCE_DEG;
        
        // Check if Flywheel has revved to the desired speed
        boolean flywheelReady = Math.abs(flywheel.getFlywheelRpm() - params.rpm) < FLYWHEEL_TOLERANCE_RPM;
        
        // Check if Angler is to desired angle for launch
        boolean anglerReady = Math.abs(angler.getPosition() - params.angle) < ANGLER_TOLERANCE_DEG;

        if(turretReady && flywheelReady && anglerReady){
            return true;
        } else {
            return false;
        }
    }
 

    // SHOT INTERPPOLATION CLASSES

    // Testing this for getAnglerAngle & flywheel rpm
    public class ShotParameter {

        public final double distance;
        public final double rpm;
        public final double angle;

        public ShotParameter(double distance, double rpm, double angle) {
            this.distance = distance;
            this.rpm = rpm;
            this.angle = angle;
        }
    }

    // List of experimental data points sorted by distance
    private final ShotParameter[] dataPoints = {
        new ShotParameter(84.0, 4900, 45),
        new ShotParameter(60.0, 4450, 40),
        new ShotParameter(35.0, 4200, 35),
        new ShotParameter(31.5, 4200, 30) 
    };

    public ShotParameter getIdealShot(double targetDistance) {

        // 1. Target is futher away than tested range
        if (targetDistance <= dataPoints[0].distance){
            return dataPoints[0];
        }

        // 2. Target is closer than tested range
        if (targetDistance <= dataPoints[dataPoints.length - 1].distance) {
            return dataPoints[dataPoints.length - 1];
        }
            
        // 3. Target is in between data points of tested range
        for (int i = 0; i < dataPoints.length - 1; i++) {
            ShotParameter p1 = dataPoints[i];
            ShotParameter p2 = dataPoints[i + 1];

            if (targetDistance >= p1.distance && targetDistance <= p2.distance) {

                // Calculate interpolation factor (0.0 to 1.0)
                double t = (targetDistance - p1.distance) / (p2.distance - p1.distance);
                
                // Interpolate RPM and Angle
                double interpolatedRpm = p1.rpm + t * (p2.rpm - p1.rpm);
                double interpolatedAngle = p1.angle + t * (p2.angle - p1.angle);

                return new ShotParameter(targetDistance, interpolatedRpm, interpolatedAngle);
            }
        }
        return dataPoints[0]; 
    }



    // SCORER COMMANDS

    // Aim method that asks each subsystem to move based on a setpoint, passes a lambda () -> to keep it live
    public Command AimToHub(){
        return turret.aimTurretToSetPointCommand(() -> getAngleToHubFromTurretPerspective())
            .alongWith( flywheel.spinFlywheelToSetPointCommand( ()-> getIdealShot(getDistanceToHub()).rpm))
            .alongWith(angler.aimAnglerToSetPointCommand(()-> getIdealShot(getDistanceToHub()).angle))
            .withName("Scorer:AimToHub");
    }



     
    public void update() {

        // update robot's pose each cycle, called in Robot.java's periodic() method
        robotX = Drivetrain.getInstance().getRobotX();
        robotY = Drivetrain.getInstance().getRobotY();
        robotAngle = Drivetrain.getInstance().getRobotAngleDegrees();
        SmartDashboard.putNumber(side.substring(0,1) + " Scorer: Distance To Hub", getDistanceToHub());
        SmartDashboard.putNumber(side.substring(0,1) + " Scorer: Angle To Hub", getAngleToHubFromTurretPerspective());
        SmartDashboard.putBoolean(side.substring(0,1) + " Scorer: READY TO FIRE", isReadyToScore());
    }




}
