package frc.robot.utils;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;


public class TestCalculations {
    
    public static void main(String[] args){

        System.out.println("\nTesting for Poses!\n------------------------------");


        // Testing Variables
        Alliance alliance = Alliance.Blue;
        int startLocation = 3;
        String element = "HUB"; //Blue Hub Front Center = 10
        

        // Print the starting point Pose coordinates (center of robot)
        printRobotPoseStart(alliance, startLocation);

        // Print the AprilTag Pose coordinates
        printTagPoseOfElement(alliance, element);

        // Print the Robot's Pose when directly facing an Element's central tag
        printRobotPoseToElement(alliance, element);


    
        // Print the value of the closest tag
        double currentX = 2.0;
        double currentY = 4.0;
        Pose3d currentPose = new Pose3d( new Pose2d(currentX, currentY, new Rotation2d(0)));
        int closestTag = FieldConstants.getNearestTag(currentPose);
        System.out.println("Closest Tag to (" + currentX + ", " + currentY + ") is " + closestTag);

        System.out.println("\n");

        // System.out.println("3 inches in meters: "+Units.inchesToMeters(3));
        // System.out.println("Gear Ratio: " + Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO);
        // System.out.println("Gear Reduction: " + Constants.SwerveModuleConstants.DRIVE_GEAR_REDUCTION);


        // PRINT OUT ALL POSES
        
        Alliance[] alliances = {Alliance.Blue, Alliance.Red};
        int[] startLocations = {1,2,3};
        String[] elements = {"LEFT_TRENCH", "HUB", "HUB2", "RIGHT_TRENCH", "OUTPOST","OUTPOST2"};

        for (Alliance a: alliances){

            // Starting Poses
            for(int startLoc : startLocations){
                printRobotPoseStart(a, startLoc);
            }

            System.out.println();

            // Robot Poses to Tags
            for(String e: elements){
                printRobotPoseToElement(a, e);
            }
            System.out.println();

            // Tag Poses
            for(String e: elements){
                printTagPoseOfElement(a, e);
            }
            System.out.println();
        }

        // PRINT OUT ALL REEF POSES
        // Pose3d targetPose = FieldConstants.getRobotPoseToBranch(tagId, branchDirection);
        // System.out.println("\n\nROBOT POSE FACING BRANCH for "+ alliance + " - Face " + reefFace + " - " + branchDirection + " branch:");
        // FieldConstants.printPose3d(targetPose);



    }


    public static void printRobotPoseStart(Alliance alliance, int startLocation){
        Pose3d startPose = FieldConstants.getRobotPoseInitial(alliance, startLocation);
        System.out.printf("\nROBOT POSE START " + alliance + " "+startLocation+":  \t");
        FieldConstants.printPose3d(startPose);
    }

    public static void printRobotPoseToElement(Alliance alliance, String element){
        int tagId = FieldConstants.getTagFromElement(alliance, element);
        Pose3d targetPose = FieldConstants.getRobotPoseFromTag(tagId);
        System.out.print("\nROBOT POSE "+ alliance + " - " + element + " (id " + tagId +"):");
        FieldConstants.printPose3d(targetPose);
    }

    public static void printTagPoseOfElement(Alliance alliance, String element){
        int tagId = FieldConstants.getTagFromElement(alliance, element);
        Pose3d tagPose = FieldConstants.getPoseFromTag(tagId);
        System.out.print("\nTAG POSE #" + tagId + " (" + alliance + "):\t");
        FieldConstants.printPose3d(tagPose);
    }




}