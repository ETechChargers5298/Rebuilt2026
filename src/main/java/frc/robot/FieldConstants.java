package frc.robot;


import java.util.Optional;
import java.util.OptionalInt;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.RobotConstants;


public class FieldConstants {

    public static int NUM_TAGS = 32;
    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final int[] FIELD_TAGS = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
    public static final double[] TAG_WEIGHTS = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

    // See dimensions in ETech CAD model: https://cad.onshape.com/documents/de2f7209733ab7e329c09bfb/w/543129b7970a07a101364ab9/e/610a7d2c02088aeabb91946e?renderMode=0&uiState=6972fd8c35795524f0b5acbd
    // Official OnShape Field: https://cad.onshape.com/documents/8a691e28680da30504859fce/w/c6aa636fb23edb3f1e272fb1/e/f4e47c668796f504844c94a0
    // ETech Spreadsheet with dimensions: https://docs.google.com/spreadsheets/d/1ADu-O-9LnCuzFm0Oi5kvYeQFNI42bzS47ktMrjclpx8/edit?gid=252485470#gid=252485470
    public static final double FIELD_LENGTH_X = 17.548;
    public static final double FIELD_WIDTH_Y = 8.052;
    public static final double FIELD_CENTER_Y = FIELD_WIDTH_Y/2; //4.026
    public static final double BLUE_WALL = 0.000;
    public static final double RED_WALL = FIELD_LENGTH_X;  
    public static final double BLUE_STARTING_LINE = 3.646; // center of robot when bumper is on top of 2" starting line
    public static final double RED_STARTING_LINE = FIELD_LENGTH_X - BLUE_STARTING_LINE;
    public static final double BLUE_LEFT_TRENCH_CENTER = 7.412;
    public static final double BLUE_DEPOT_CENTER = 5.965;
    public static final double BLUE_LEFT_BUMP_CENTER = 5.571;
    public static final double BLUE_HUB_CENTER = FIELD_CENTER_Y;
    public static final double BLUE_RIGHT_BUMP_CENTER = 2.481;
    public static final double BLUE_RIGHT_TRENCH_CENTER = 0.639;
    public static final double RED_LEFT_TRENCH_CENTER = BLUE_RIGHT_TRENCH_CENTER;
    public static final double RED_DEPOT_CENTER = FIELD_WIDTH_Y - BLUE_DEPOT_CENTER;
    public static final double RED_LEFT_BUMP_CENTER = BLUE_RIGHT_BUMP_CENTER;
    public static final double RED_HUB_CENTER = FIELD_CENTER_Y;
    public static final double RED_RIGHT_BUMP_CENTER = BLUE_LEFT_BUMP_CENTER;
    public static final double RED_RIGHT_TRENCH_CENTER = BLUE_LEFT_TRENCH_CENTER;
    public static final double BLUE_AUTO_ANGLE = 0;
    public static final double RED_AUTO_ANGLE = 180;

    public static final Translation2d fieldCenter = new Translation2d(FIELD_LENGTH_X/2, FIELD_WIDTH_Y/2);

    // Center-of-Robot Poses
    public static final Pose3d blueHubStartPose = new Pose3d(new Pose2d(BLUE_STARTING_LINE, BLUE_HUB_CENTER, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d blueLeftTrenchStartPose = new Pose3d( new Pose2d(BLUE_STARTING_LINE, BLUE_LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d blueDepotStartPose = new Pose3d( new Pose2d(BLUE_STARTING_LINE, BLUE_DEPOT_CENTER, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d redHubStartPose = new Pose3d(new Pose2d(RED_STARTING_LINE, RED_HUB_CENTER, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));
    public static final Pose3d redLeftTrenchStartPose = new Pose3d( new Pose2d(RED_STARTING_LINE, RED_LEFT_TRENCH_CENTER, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));
    public static final Pose3d redDepotStartPose = new Pose3d( new Pose2d(RED_STARTING_LINE, RED_DEPOT_CENTER, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));
    public static final Pose3d blueDepotGatherPose = new Pose3d( new Pose2d(BLUE_WALL, BLUE_DEPOT_CENTER, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d redDepotGatherPose = new Pose3d( new Pose2d(RED_WALL, RED_DEPOT_CENTER, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));
    public static final Pose3d blueRightTrenchStartPose = new Pose3d( new Pose2d(BLUE_STARTING_LINE, BLUE_RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d redRightTrenchStartPose = new Pose3d( new Pose2d(RED_STARTING_LINE, RED_RIGHT_TRENCH_CENTER, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));


    //--------------- ROBOT CENTER POSE METHODS ------------------//

     // Returns a robot's center pose when facing a specific aprilTag
     public static Pose3d getRobotPoseFromTag(int tagID){

        Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagID).get();

        //Get the coordinate of tag on field
        Translation3d tagTranslation = tagPose.getTranslation();
        double tagAngle = tagPose.getRotation().getAngle();

        Pose3d flippedPose = getRobotPoseSpin180(tagPose);
        Pose3d centerOfRobotPose = getCenterPoseFromFrontPose(flippedPose, 0.0);
        return centerOfRobotPose;
    }


    // Finds the Pose3d 180 degrees rotated from original pose, staying in same coordinates
    public static Pose3d getRobotPoseSpin180(Pose3d originalPose){
        return originalPose.rotateAround(originalPose.getTranslation(), new Rotation3d(0,0,Units.degreesToRadians(180)));
    }

    // Provides a Pose3d to the center of the robot given a Pose3d at the front of the bumper
    public static Pose3d getCenterPoseFromFrontPose(Pose3d frontPose, double offset){

        // System.out.println("\n\nStarting with frontPose of: "+frontPose + "---->");
        double frontToCenterDistance = RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE;
        // System.out.println("\tFTCD (meters): " + frontToCenterDistance);
        double angleRadians = frontPose.getRotation().getZ();
        // System.out.printf("\tAngleDegrees: \t%.1f\n", Math.toDegrees(frontPose.getRotation().getZ()));

        double xOffset = (frontToCenterDistance - offset) * Math.cos(angleRadians);
        double yOffset = (frontToCenterDistance - offset) * Math.sin(angleRadians);
        // System.out.println("\tMath.cos(angleRad) = " + Math.cos(angleRadians));
        // System.out.println("\tMath.sin(angleRad) = " + Math.sin(angleRadians));
        // System.out.println("\txoffset: "+xOffset + ", yOffset: " + yOffset);

        Translation3d translationOffset = new Translation3d(xOffset, yOffset,0);
        Translation3d translationNew = frontPose.getTranslation().minus(translationOffset);
        // System.out.println("\ttoffset: " + translationOffset);
        // System.out.println("\ttnew: " + translationNew);

        Pose3d centerPose = new Pose3d(translationNew, frontPose.getRotation());
        // System.out.println("--->Returning centerPose of: "+ centerPose + "/n/n");

        return centerPose;
    }
    
    // Gets our initial Pose based on the FMS alliance/location
    // Location 1 is Left Side (Depot)
    // location 2 is in the Middle (Hub)
    // location 3 is Right Side (Outpost)
    public static Pose3d getRobotPoseInitialFMS(){

        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        Alliance alliance = null;
        if (allianceOptional.isPresent()) {
            alliance = allianceOptional.get();
        }    

        OptionalInt locationOptional = DriverStation.getLocation();
        int location = -1;        
        if(locationOptional.isPresent()){
            location = locationOptional.getAsInt();
        }

        return getRobotPoseInitial(alliance, location);
    }
    
    // Returns the Robot center's pose given specific starting positions
    public static Pose3d getRobotPoseInitial(Alliance alliance, int location){
        
        if (location == 1 && alliance == DriverStation.Alliance.Blue) { return blueDepotStartPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Blue) { return blueHubStartPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Blue) { return blueRightTrenchStartPose; }
        else if (location == 1 && alliance == DriverStation.Alliance.Red) { return redDepotStartPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Red) { return redHubStartPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Red) { return redRightTrenchStartPose; }

        return new Pose3d();
    }

    // Returns the Pose3d of an AprilTag given its ID number
    public static Pose3d getPoseFromTag(int tagID){
        return aprilTagFieldLayout.getTagPose(tagID).get();
    }

        // Returns the nearest reef apriltag from an input pose
    public static int getNearestTag(Pose3d currentRobotPose) {

        double minDistance = 0.0;
        int closestTag = -1;

        for(int tag: FIELD_TAGS){
            
            Pose3d facePose = aprilTagFieldLayout.getTagPose(tag).get();
            double dx = facePose.getX() - currentRobotPose.getX();
            double dy = facePose.getY() - currentRobotPose.getY();
            double distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));

            if(distance < minDistance){
                minDistance = distance;
                closestTag = tag;
            }
        }
        return closestTag;
    }



    // Determine the tag at the desired Coral Station
    public static int getTagFromElement(Alliance alliance, String element){

        if (alliance == Alliance.Blue && element.equals("LEFT_TRENCH")) return 23;
        else if (alliance == Alliance.Blue && element.equals("HUB")) return 26;
        else if (alliance == Alliance.Blue && element.equals("HUB2")) return 25;
        else if (alliance == Alliance.Blue && element.equals("RIGHT_TRENCH")) return 28;
        else if (alliance == Alliance.Blue && element.equals("OUTPOST")) return 29;
        else if (alliance == Alliance.Blue && element.equals("OUTPOST2")) return 30;

        else if (alliance == Alliance.Red && element.equals("LEFT_TRENCH")) return 7;
        else if (alliance == Alliance.Red && element.equals("HUB")) return 10;
        else if (alliance == Alliance.Red && element.equals("HUB2")) return 9;
        else if (alliance == Alliance.Red && element.equals("RIGHT_TRENCH")) return 12;
        else if (alliance == Alliance.Red && element.equals("OUTPOST")) return 13;
        else if (alliance == Alliance.Red && element.equals("OUTPOST2")) return 14;

        else return -1;
    }

    public static void printPose3d(Pose3d p3d){
        System.out.printf("\tX: \t%.3f", p3d.getX());
        System.out.printf("\tY: \t%.3f", p3d.getY());
        System.out.printf("\tAngle: \t%.1f", Math.toDegrees(p3d.getRotation().getZ()));
    }

    
}