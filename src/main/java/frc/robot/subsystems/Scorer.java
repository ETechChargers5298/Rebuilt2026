package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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


        return 0.0;
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



}
