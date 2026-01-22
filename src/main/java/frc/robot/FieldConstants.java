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

    public static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public static final int[] REEF_TAGS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static final double[] TAG_WEIGHTS = { 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1, 0.25, 0.25, 0.25, 0.25, 0.25, 1, 1, 1, 1, 1, 1};

    // See dimensions in CAD model: https://cad.onshape.com/documents/73436e28519cd6ef4a2eaa1e/w/658277bde6080a5805de078e/e/42ebe4fcf42b8b1c992b38c2
    public static final double FIELD_LENGTH_X = 17.548;
    public static final double FIELD_WIDTH_Y = 8.052;
    public static final double BLUE_CENTER_CAGE_Y = 6.169;
    public static final double RED_CENTER_CAGE_Y = 1.883;
    public static final double FIELD_CENTER_Y = FIELD_WIDTH_Y/2; //4.026
    public static final double BLUE_STARTING_LINE = 7.606; //"back" edge of 2" starting line
    public static final double BLUE_STARTING_X = BLUE_STARTING_LINE - RobotConstants.BUMPER_TO_ROBOT_CENTER_DISTANCE;  //7.174 Center of Robot when Starting
    public static final double RED_STARTING_X = FIELD_LENGTH_X - BLUE_STARTING_X;
    public static final double BLUE_AUTO_ANGLE = 180;
    public static final double RED_AUTO_ANGLE = 0;

    public static final Translation2d fieldCenter = new Translation2d(FIELD_LENGTH_X/2, FIELD_WIDTH_Y/2);

    // Center-of-Robot Poses from AUTO Starting Positions
    public static final Pose3d blueBargeSideAutoPose = new Pose3d(new Pose2d(BLUE_STARTING_X, BLUE_CENTER_CAGE_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d blueCenterAutoPose = new Pose3d( new Pose2d(BLUE_STARTING_X, FIELD_CENTER_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d blueProcessorSideAutoPose = new Pose3d( new Pose2d(BLUE_STARTING_X, RED_CENTER_CAGE_Y, Rotation2d.fromDegrees(BLUE_AUTO_ANGLE)));
    public static final Pose3d redBargeSideAutoPose = new Pose3d( new Pose2d(RED_STARTING_X, RED_CENTER_CAGE_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));
    public static final Pose3d redCenterAutoPose = new Pose3d( new Pose2d(RED_STARTING_X, FIELD_CENTER_Y, Rotation2d.fromDegrees(RED_AUTO_ANGLE)));
    public static final Pose3d redProcessorSideAutoPose = new Pose3d(new Pose2d(RED_STARTING_X, BLUE_CENTER_CAGE_Y, Rotation2d.fromDegrees(-45)));

    // Center-of-Robot Poses facing CORAL STATION
    public static final Pose3d blueCoralStationLeftPose = getRobotPoseToCoralStation(Alliance.Blue,"LEFT");
    public static final Pose3d blueCoralStationRightPose = getRobotPoseToCoralStation(Alliance.Blue,"RIGHT");
    public static final Pose3d redCoralStationLeftPose = getRobotPoseToCoralStation(Alliance.Red,"LEFT");
    public static final Pose3d redCoralStationRightPose = getRobotPoseToCoralStation(Alliance.Red,"RIGHT");


    //--------------- ROBOT CENTER POSE METHODS ------------------//

    // Returns a robot's center pose to a specific branch
    public static Pose3d getRobotPoseToBranch(int tagID, String branchDirection){
        Pose3d branchPose = getBranchPose(tagID, branchDirection); 
        Pose3d flippedPose = getRobotPoseSpin180(branchPose);
        Pose3d centerOfRobotPose = getCenterPoseFromFrontPose(flippedPose, Units.inchesToMeters(3));
        return centerOfRobotPose;
    }

    // Returns a robot's center pose to a specific coral station
    public static Pose3d getRobotPoseToCoralStation(Alliance alliance, String stationDirection){
        Pose3d coralStationPose = getCoralStationPose(alliance, stationDirection);
        // System.out.print("\n\nCoral Station Pose:");
        // printPose3d(coralStationPose);

        Pose3d flippedPose = getRobotPoseSpin180(coralStationPose);
        // System.out.print("\nFlippedPose:    ");
        // printPose3d(flippedPose);

        Pose3d centerOfRobotPose = getCenterPoseFromFrontPose(flippedPose, 0.0);
        // System.out.print("\ncenterRobotPose:");
        // printPose3d(centerOfRobotPose);

        Pose3d facingBackwardsPose = getRobotPoseSpin180(centerOfRobotPose); //have robot facing away from the Coral Station to load
        // System.out.print("\nfacingBackwards:");
        // printPose3d(facingBackwardsPose);

        return facingBackwardsPose;
    }

     // Returns a robot's center pose when facing a specific aprilTag
     public static Pose3d getRobotPoseToTag(int tagID){

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
    // Location 1 is Left Side (Barge)
    // location 2 is in the Middle
    // location 3 is Right Side (Processor)
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
        
        if (location == 1 && alliance == DriverStation.Alliance.Blue) { return blueBargeSideAutoPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Blue) { return blueCenterAutoPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Blue) { return blueProcessorSideAutoPose; }
        else if (location == 1 && alliance == DriverStation.Alliance.Red) { return redBargeSideAutoPose; }
        else if (location == 2 && alliance == DriverStation.Alliance.Red) { return redCenterAutoPose; }
        else if (location == 3 && alliance == DriverStation.Alliance.Red) { return redProcessorSideAutoPose; }

        return new Pose3d();
    }

    //--------------- FIELD ELEMENT POSE METHODS ------------------//

    // Returns the Pose3d of a BRANCH (facing out of the Reef), adjusting for an offset from the aprilTag on that face
    public static Pose3d getBranchPose(int tagID, String branchDirection){

        Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagID).get();

        //Get the coordinate of tag on field
        Translation3d tagTranslation = tagPose.getTranslation();
        double tagAngle = tagPose.getRotation().getAngle();

        //Create offset from tag to desired branch
        Translation3d branchOffset = new Translation3d();
        double calibrationDistance = Units.inchesToMeters(0.5); 
        double branchOffsetDistance = Units.inchesToMeters(6.5) + calibrationDistance;
        
        if (branchDirection.equals("LEFT")){
            branchOffset = new Translation3d(Math.sin(tagAngle) * branchOffsetDistance, -Math.cos(tagAngle) * branchOffsetDistance, 0);
        }  else  if (branchDirection.equals("RIGHT")){
            branchOffset = new Translation3d(-Math.sin(tagAngle) * branchOffsetDistance, Math.cos(tagAngle) * branchOffsetDistance, 0);
        } else if(branchDirection.equals("CENTER")){
            //no branch offset
        }

        // Add tag and branch offset to get desired target coordinate --> target "Pose" to front of Bumper
        Translation3d targetCoordinate = branchOffset.plus(tagTranslation);
        Pose3d targetPose = new Pose3d(targetCoordinate, tagPose.getRotation());
        
        return targetPose;
    }

    // Returns the Pose3d of a Robot backin up to the CORAL STATION
    public static Pose3d getCoralStationPose(Alliance alliance, String stationDirection){

        // Determine the tag at the desired Coral Station
        int tagID = getTagFromCoralStation(alliance, stationDirection);

        // Get the Pose3d of the Coral Station
        Pose3d tagPose = aprilTagFieldLayout.getTagPose(tagID).get();

        //Get the coordinate of tag on field
        Translation3d tagTranslation = tagPose.getTranslation();
        double tagAngle = tagPose.getRotation().getAngle();

        //Create offset from tag to easier coral slot location?
        Translation3d coralslotOffset = new Translation3d();
        double coralslotOffsetDistance = Units.inchesToMeters(0.0);
        
        if (stationDirection.equals("LEFT")){
            coralslotOffset = new Translation3d(Math.sin(tagAngle) * coralslotOffsetDistance, -Math.cos(tagAngle) * coralslotOffsetDistance, 0);
        }  else  if (stationDirection.equals("RIGHT")){
            coralslotOffset = new Translation3d(-Math.sin(tagAngle) * coralslotOffsetDistance, Math.cos(tagAngle) * coralslotOffsetDistance, 0);
        }

        // Add tag and branch offset to get desired target coordinate --> target Robot Pose at center of Robot
        Translation3d targetCoordinate = coralslotOffset.plus(tagTranslation);
        Pose3d targetPose = new Pose3d(targetCoordinate, tagPose.getRotation());
        
        return targetPose;
    }

    // Returns the Pose3d of an AprilTag given its ID number
    public static Pose3d getTagPose(int tagID){
        return aprilTagFieldLayout.getTagPose(tagID).get();
    }


    //--------------- TAG ID METHODS ------------------//

    // Returns the nearest reef apriltag from an input pose
    public static int getNearestReefTag(Pose3d currentRobotPose) {

        double minDistance = 30.0;
        int closestTag = -1;

        for(int reefTag: REEF_TAGS){
            
            Pose3d reefFacePose = aprilTagFieldLayout.getTagPose(reefTag).get();
            double dx = reefFacePose.getX() - currentRobotPose.getX();
            double dy = reefFacePose.getY() - currentRobotPose.getY();
            double distance = Math.sqrt(Math.pow(dx,2)+Math.pow(dy,2));

            if(distance < minDistance){
                minDistance = distance;
                closestTag = reefTag;
            }
        }
        return closestTag;
    }

    // See AprilTag field map: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/Apriltag_Images_and_User_Guide.pdf
    public static int getTagFromReef(Alliance alliance, String reefFace){
        
        if(reefFace.equals("A") && alliance == Alliance.Blue) return 18;
        else if(reefFace.equals("B") & alliance == Alliance.Blue) return 17;
        else if(reefFace.equals("C") & alliance == Alliance.Blue) return 22;
        else if(reefFace.equals("D") & alliance == Alliance.Blue) return 21;
        else if(reefFace.equals("E") & alliance == Alliance.Blue) return 20;
        else if(reefFace.equals("F") & alliance == Alliance.Blue) return 19;
        else if(reefFace.equals("A") && alliance == Alliance.Red) return 7;
        else if(reefFace.equals("B") & alliance == Alliance.Red) return 8;
        else if(reefFace.equals("C") & alliance == Alliance.Red) return 9;
        else if(reefFace.equals("D") & alliance == Alliance.Red) return 10;
        else if(reefFace.equals("E") & alliance == Alliance.Red) return 11;
        else if(reefFace.equals("F") & alliance == Alliance.Red) return 6;
        else return -1;
    }

    // Determine the tag at the desired Coral Station
    public static int getTagFromCoralStation(Alliance alliance, String stationDirection){

        if (alliance == Alliance.Blue && stationDirection.equals("LEFT")) return 13;
        else if (alliance == Alliance.Blue && stationDirection.equals("RIGHT")) return 12;
        else if (alliance == Alliance.Red && stationDirection.equals("LEFT")) return 1;
        else if (alliance == Alliance.Red && stationDirection.equals("RIGHT")) return 2;
        else return -1;
    }

    public static void printPose3d(Pose3d p3d){
        System.out.printf("\tX: \t%.3f", p3d.getX());
        System.out.printf("\tY: \t%.3f", p3d.getY());
        System.out.printf("\tAngle: \t%.1f", Math.toDegrees(p3d.getRotation().getZ()));
    }

    
}