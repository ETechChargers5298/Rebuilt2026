package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Constants.*;


public class Scorer extends SubsystemBase {

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
    private double hubY = 0;
    private double depotHerdX = 0;
    private double depotHerdY = 0;
    private double outpostHerdX = 0;
    private double outpostHerdY = 0;
    private double bonusMeasure = 0;
    private double bonusDistance = 0.2;

    private double targetX = 0;
    private double targetY = 0;
    private String targetString = "";
    private boolean movingFlag = false;
    


    double robotX = Drivetrain.getInstance().getRobotX();
    double robotY = Drivetrain.getInstance().getRobotY();
    double robotAngle = Drivetrain.getInstance().getRobotAngleDegrees();
    Optional<Alliance> allianceOptional = DriverStation.getAlliance();
    static Alliance alliance = null;
    

    // SCORER CONSTRUCTOR
    public Scorer(String side, double xOffset, double yOffset){
        
        // Define subsystems for left & right scorers
        this.side = side;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
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


        // Define field coordinates depending on Alliance color
        if (allianceOptional.isPresent()) {
            alliance = allianceOptional.get();
        }

        if(alliance == DriverStation.Alliance.Blue){
            hubX = FieldConstants.BLUE_HUB_CENTER_X;
            hubY = FieldConstants.FIELD_CENTER_Y;
            depotHerdX = FieldConstants.BLUE_DEPOT_HERD_X;
            depotHerdY = FieldConstants.BLUE_DEPOT_HERD_Y;
            outpostHerdX = FieldConstants.BLUE_OUTPOST_HERD_X;
            outpostHerdY = FieldConstants.BLUE_OUTPOST_HERD_Y;

        } else {
            hubX = FieldConstants.RED_HUB_CENTER_X;
            hubY = FieldConstants.FIELD_CENTER_Y;
            depotHerdX = FieldConstants.RED_DEPOT_HERD_X;
            depotHerdY = FieldConstants.RED_DEPOT_HERD_Y;
            outpostHerdX = FieldConstants.RED_OUTPOST_HERD_X;
            outpostHerdY = FieldConstants.RED_OUTPOST_HERD_Y;
        }

        // Set the hub as the default target
        targetX = hubX;
        targetY = hubY;
        targetString = "Hub";

        SmartDashboard.putData("toggleMovingFlag", flagToggleCommand());
    }



    // SCORER METHODS

    public void setTarget(double targetX, double targetY){
        this.targetX = targetX;
        this.targetY = targetY;
    }



    // Methods to change the target
    public void setTargetToHub(){

        if(!movingFlag){
            setTarget(hubX, hubY);
            targetString = "Hub";
        }
        else{

            // https://docs.google.com/spreadsheets/d/1GuRmCdJDzk8IGj2cKh8zotB-9alPd8FhiDb3VZPBC_4/edit?usp=sharing
            double distance = getDistanceToHub();
            double timeOfFlight = 0.0539 * distance + 0.854;
            double sorcererX = hubX + Drivetrain.getInstance().getRobotVelocityToFieldX() * timeOfFlight;
            double sorcererY = hubY + Drivetrain.getInstance().getRobotVelocityToFieldY() * timeOfFlight;

            setTarget(sorcererX, sorcererY);
            targetString = "HubMoving";
        }
    }
    public void setTargetToHerdDepot(){
        setTarget(depotHerdX, depotHerdY);
        targetString = "Depot";
    }
        public void setTargetToHerdOutpost(){
        setTarget(outpostHerdX, outpostHerdY);
        targetString = "Outpost";
    }


    // Methods to change the perceived distance needed for a fuel launch
    public void bonusUp(double increase) {
        bonusMeasure += increase;
    }
    public void bonusDown(double increase) {
        bonusMeasure -= increase;
    }


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

    // Distance from TurretCenter -->  A Target
    public double getDistanceToTarget(){
        double turretX = getTurretX();
        double turretY = getTurretY();
        return Math.sqrt(Math.pow(targetX - turretX, 2) + Math.pow(targetY - turretY, 2));
    }


    // [DEPRECCATED] Distance from TurretCenter -->  Alliance Hub Center
    public double getDistanceToHub(){
        double tx = getTurretX();
        double ty = getTurretY();
        return Math.sqrt(Math.pow(hubX - tx, 2) + Math.pow(hubY - ty, 2));
    }

    // Angle from HubCenter --> TargetCenter --> Field X-axis 0
    public double getAngleToTargetFromFieldPerspective(){
        return Math.toDegrees(Math.atan2(targetY - getTurretY(), targetX - getTurretX()));
    }

    //  [DEPRECATED] Angle from HubCenter --> TurretCenter --> Field X-axis 0
    public double getAngleToHubFromFieldPerspective(){
        return Math.toDegrees(Math.atan2(hubY - getTurretY(), hubX - getTurretX()));
    }

    //  Angle from HubCenter --> TargetCenter --> RobotFront X-axis (front/intake)
    public double getAngleToTargetFromRobotPerspective(){
        return getAngleToTargetFromFieldPerspective() - robotAngle;
    }
 
    //  [DEPRECATED] Angle from HubCenter --> TurretCenter --> RobotFront X-axis (front/intake)
    public double getAngleToHubFromRobotPerspective(){
        return getAngleToHubFromFieldPerspective() - robotAngle;
    }

    // Angle from HubCenter - ScorerCenter - Turret X-axis
    // if you are at the turret's center (facing backward/starting angle), to which angle is the hub?
    public double getAngleToTargetFromTurretPerspective(){
        return (180.0 - getAngleToTargetFromRobotPerspective() - 360);
    }


    //  [DEPRECATED] Angle from HubCenter - ScorerCenter - Turret X-axis
    // if you are at the turret's center (facing backward/starting angle), to which angle is the hub?
    public double getAngleToHubFromTurretPerspective(){
        return 180.0 - getAngleToHubFromRobotPerspective() - 360;
    }

    //will toggle movingFlag
    public void toggleFlag (){
        movingFlag = !movingFlag;
    }

    // check if the Scorer mechs are ready for a launch
    public boolean isReadyToScore() {
        
        double dist = getDistanceToTarget();
        var params = getIdealShot(dist);
        
        // Check if Turret is aimed towards Hub
        boolean turretReady = Math.abs(turret.getTurretAngle() - getAngleToTargetFromTurretPerspective()) < ScorerConstants.TURRET_TOLERANCE_DEG;
        
        // Check if Flywheel has revved to the desired speed
        // TODO: Get max flywheel RPM and divide so that we can compare to setSpeed
        boolean flywheelReady = true; // Math.abs(flywheel.getFlywheelRpm() - params.setSpeed) < ScorerConstants.FLYWHEEL_TOLERANCE_RPM;
        
        // Check if Angler is to desired angle for launch
        boolean anglerReady = Math.abs(angler.getPosition() - params.angle) < ScorerConstants.ANGLER_TOLERANCE_DEG;

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
        public final double setSpeed;
        public final double angle;

        public ShotParameter(double distance, double setSpeed, double angle) {
            this.distance = distance;
            this.setSpeed = setSpeed;
            this.angle = angle;
        }
    }

    // List of experimental data points sorted by distance
    private final ShotParameter[] dataPoints = {
        new ShotParameter(1.81, -0.75, 0),
        new ShotParameter(2.86, -0.95, 0),
        new ShotParameter(3.35, -1, -2.5)
    };

    public ShotParameter getIdealShot(double targetDistance) {

        // 1. Target is closer than tested range
        if (targetDistance <= dataPoints[0].distance){
            return dataPoints[0];
        }

        // 2. Target is further than tested range
        if (targetDistance >= dataPoints[dataPoints.length - 1].distance) {
            return dataPoints[dataPoints.length - 1];
        }
            
        // 3. Target is in between data points of tested range
        for (int i = 0; i < dataPoints.length - 1; i++) {
            ShotParameter p1 = dataPoints[i];
            ShotParameter p2 = dataPoints[i + 1];

            if (p1.distance <= targetDistance && targetDistance <= p2.distance) {

                // Calculate interpolation factor (0.0 to 1.0)
                double t = (targetDistance - p1.distance) / (p2.distance - p1.distance);
                
                // Interpolate setspeed and Angle
                double interpolatedSetSpeed = p1.setSpeed + t * (p2.setSpeed - p1.setSpeed);
                double interpolatedAngle = p1.angle + t * (p2.angle - p1.angle);

                return new ShotParameter(targetDistance, interpolatedSetSpeed, interpolatedAngle);
            }
        }
        return dataPoints[0]; 
    }

    // SCORER COMMANDS

    // Increases power of launcher incrementally
    public Command bonusUpCommand(){
        return runOnce(
            () -> {
            bonusUp(bonusDistance);
        });
    }

    // Decreases power of launcher incrementally
    public Command bonusDownCommand(){
        return runOnce(
            () -> {
            bonusDown(bonusDistance);
        });
    }

    //command for toggleFlag
    public Command flagToggleCommand(){
            Command toggleFlag = new InstantCommand(()-> toggleFlag());
        return toggleFlag.ignoringDisable(true);
    }
    // Aim method that asks each subsystem to move based on a setpoint, passes a lambda () -> to keep it live
    public Command AimToTarget(){
        return turret.aimTurretToSetPointCommand(() -> getAngleToTargetFromTurretPerspective())
            .alongWith( flywheel.flyWheelCommand(() -> getIdealShot(getDistanceToTarget() + bonusMeasure ).setSpeed))
            .alongWith(angler.aimAnglerToSetPointCommand(()-> getIdealShot(getDistanceToTarget() + bonusMeasure).angle))
            .withName("Scorer:AimToTarget");
    }
     
    public void update() {
        // update robot's pose each cycle, called in Robot.java's periodic() method
        robotX = Drivetrain.getInstance().getRobotX();
        robotY = Drivetrain.getInstance().getRobotY();
        robotAngle = Drivetrain.getInstance().getRobotAngleDegrees();
        ShotParameter shots = getIdealShot(getDistanceToTarget());
        SmartDashboard.putNumber(side.substring(0,1) + " Scorer: Distance To Target", getDistanceToTarget());
        SmartDashboard.putNumber(side.substring(0,1) + " Scorer: Angle To Target", getAngleToTargetFromTurretPerspective());
        SmartDashboard.putBoolean(side.substring(0,1) + " Scorer: READY TO FIRE", isReadyToScore());
        SmartDashboard.putString(side.substring(0,1)  + " Current Target", targetString);
        SmartDashboard.putNumber(side.substring(0,1)  + " TargetX", targetX);
        SmartDashboard.putNumber(side.substring(0,1)  + " TargetY", targetY);
        SmartDashboard.putNumber(side.substring(0, 1) + " Ideal Shot Setspeed", shots.setSpeed);
        SmartDashboard.putNumber(side.substring(0, 1) + " Ideal Shot Angle (Degrees)", shots.angle);
        SmartDashboard.putNumber(side.substring(0,1) + " Scorer: Bonus Distance", this.bonusMeasure );
        SmartDashboard.putBoolean(side.substring(0,1)+ "Moving While Shooting", movingFlag);
    }   
}
