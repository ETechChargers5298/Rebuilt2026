package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
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
    private double hubX = FieldConstants.BLUE_HUB_CENTER_X; 
    private double hubY = FieldConstants.FIELD_CENTER_Y; //meters
    private double robotX = Drivetrain.getInstance().getRobotX();
    private double robotY = Drivetrain.getInstance().getRobotY();
    private double robotAngle = Drivetrain.getInstance().getRobotAngleDegrees();
    

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

    }


    // SCORER METHODS

 /*    public Command aimToHub(){
        return new RunCommand(() -> {
            
            double dist = getDistanceToHub();
            ShotParameter params = getIdealShot(dist);

            double targetFieldAngle = getPredictedTargetAngle();

            turret.setFieldRelativeAngle(targetFieldAngle);

            flywheel.setReferenceRPM(params.rpm);
            angler.setAngle(params.angle);

        }, turret, flywheel, angler);
    }
*/
    // Distance from TurretCenter -->  Alliance Hub Center
    public double getDistanceToHub(){
        
        return Math.sqrt(Math.pow(hubX-robotX,2)+Math.pow(hubY-robotY,2));
    }

    // Angle from HubCenter --> TurretCenter --> Field X-axis 0
    public double getAngleToHubFromFieldPerspective(){
        double turretX = robotX + xOffset;
        double turretY = robotY + yOffset;
        return Math.toDegrees(Math.atan2(hubY - turretY, hubX - turretX));
    }
 
    // Angle from HubCenter --> TurretCenter --> RobotFront X-axis (front/intake)
    public double getAngleToHubFromRobotPerspective(){


        return getAngleToHubFromFieldPerspective() - robotAngle;
    }

    // Angle from HubCenter - ScorerCenter - Turret X-axis
    // if you are at the turret's center (facing backward/starting angle), to which angle is the hub?
    public double getAngleToHubFromTurretPerspective(){
        return 180.0 - getAngleToHubFromRobotPerspective();
    }

    
    // Method to be called once per scheduler run
    public void periodic() {
        SmartDashboard.putNumber("Distance To Hub", getDistanceToHub());
        SmartDashboard.putNumber("Angle To Hub - Turret", getAngleToHubFromTurretPerspective());
    }

    //testing this for getAnglerAngle & flywheel rpm
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

public class ShooterInterpolation { 
    // Your 6 data points sorted by distance
    private final ShotParameter[] dataPoints = {
        new ShotParameter(84.0, 4900, 45),
        new ShotParameter(60.0, 4450, 40),
        new ShotParameter(35.0, 4200, 35),
        new ShotParameter(31.5, 4200, 30)
        
    };

    public ShotParameter getIdealShot(double targetDistance) {
        // 1. Handle edge cases (distance outside your measured range)
        if (targetDistance <= dataPoints[0].distance) return dataPoints[0];
        if (targetDistance >= dataPoints[dataPoints.length - 1].distance) 
            return dataPoints[dataPoints.length - 1];

        // 2. Find the two points we are between
        for (int i = 0; i < dataPoints.length - 1; i++) {
            ShotParameter p1 = dataPoints[i];
            ShotParameter p2 = dataPoints[i + 1];

            if (targetDistance >= p1.distance && targetDistance <= p2.distance) {
                // 3. Calculate interpolation factor (0.0 to 1.0)
                double t = (targetDistance - p1.distance) / (p2.distance - p1.distance);
                
                // 4. Interpolate RPM and Angle
                double interpolatedRpm = p1.rpm + t * (p2.rpm - p1.rpm);
                double interpolatedAngle = p1.angle + t * (p2.angle - p1.angle);

                return new ShotParameter(targetDistance, interpolatedRpm, interpolatedAngle);
            }
        }
        return dataPoints[0]; // Fallback
    }
}

}
