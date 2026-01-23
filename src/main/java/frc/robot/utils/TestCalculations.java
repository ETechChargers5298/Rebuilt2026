package frc.robot.utils;

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
        String reefFace = "C";  //Blue D = 21
        String branchDirection = "LEFT";
        String stationDirection = "RIGHT";
        

        // Print the starting point Pose coordinates (center of robot)
        printPoseRobotStart(alliance, startLocation);
        
        // Print the AprilTag Reef Pose coordinates
        printPoseTagReef(alliance, reefFace);

        // Print the Branch pose coordinates (center of robot)
        printPoseRobotToBranch(alliance, reefFace, branchDirection);

        // Print the Branch's Pose
        // printPoseBranch(alliance, reefFace, branchDirection);

        // Print the AprilTag CS Pose coordinates
        printPoseTagCoralStation(alliance, stationDirection);

        // Print the Robot's Pose when Gathering from Coral Station
        printPoseRobotToCoralStation(alliance, stationDirection);


    
        // Print the value of the closest tag
        double currentX = 2.0;
        double currentY = 4.0;
        Pose3d currentPose = new Pose3d( new Pose2d(currentX, currentY, new Rotation2d(0)));
        int closestTag = FieldConstants.getNearestReefTag(currentPose);
        System.out.println("Closest Tag to (" + currentX + ", " + currentY + ") is " + closestTag);

        String branchDirFromButton = "LEFT"; 
        Pose3d closestBranchPose = FieldConstants.getRobotPoseToBranch(closestTag, branchDirFromButton);
        System.out.println("\n\nROBOT POSE FACING CLOSEST BRANCH from (" + currentX + "," + currentY + ") is at "+ alliance + " - tag " + closestTag + " - " + branchDirFromButton + " branch:");
        FieldConstants.printPose3d(closestBranchPose);


        System.out.println("\n");

        // System.out.println("3 inches in meters: "+Units.inchesToMeters(3));
        // System.out.println("Gear Ratio: " + Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO);
        // System.out.println("Gear Reduction: " + Constants.SwerveModuleConstants.DRIVE_GEAR_REDUCTION);


        // PRINT OUT ALL POSES
        
        Alliance[] alliances = {Alliance.Blue, Alliance.Red};
        int[] startLocations = {1,2,3};
        String[] reefFaces = {"A", "B", "C", "D", "E","F"};

        for (Alliance a: alliances){

            // Starting Poses
            for(int startLoc : startLocations){
                printPoseRobotStart(a, startLoc);
            }

            System.out.println();

            // Reef Face Poses
            for(String face: reefFaces){
                printPoseRobotToBranch(a,face, "LEFT");
                printPoseRobotToBranch(a, face, "CENTER");
                printPoseRobotToBranch(a,face, "RIGHT");
            }
            System.out.println();

            // Coral Station Poses
            printPoseTagCoralStation(a, "LEFT");
            printPoseRobotToCoralStation(a, "LEFT");
            printPoseTagCoralStation(a, "RIGHT");
            printPoseRobotToCoralStation(a, "RIGHT");
            System.out.println();
        }

        // PRINT OUT ALL REEF POSES
        // Pose3d targetPose = FieldConstants.getRobotPoseToBranch(tagId, branchDirection);
        // System.out.println("\n\nROBOT POSE FACING BRANCH for "+ alliance + " - Face " + reefFace + " - " + branchDirection + " branch:");
        // FieldConstants.printPose3d(targetPose);



    }


    public static void printPoseRobotStart(Alliance alliance, int startLocation){
        Pose3d startPose = FieldConstants.getRobotPoseInitial(alliance, startLocation);
        System.out.printf("\nROBOT POSE START " + alliance + " "+startLocation+":  \t");
        FieldConstants.printPose3d(startPose);
    }

    // public static void printPoseRobotToCenterReef(Alliance alliance, String reefFace){
    //     int tagId = FieldConstants.getTagFromReef(alliance, reefFace);
    //     Pose3d targetPose = FieldConstants.getRobotPoseTo(tagId, branchDirection);
    //     System.out.print("\nROBOT POSE BRANCH "+ alliance + " - " + reefFace + " - " + branchDirection + ":");
    //     FieldConstants.printPose3d(targetPose);
    // }

    public static void printPoseRobotToBranch(Alliance alliance, String reefFace, String branchDirection){
        int tagId = FieldConstants.getTagFromReef(alliance, reefFace);
        Pose3d targetPose = FieldConstants.getRobotPoseToBranch(tagId, branchDirection);
        System.out.print("\nROBOT POSE BRANCH "+ alliance + " - " + reefFace + " - " + branchDirection + ":");
        FieldConstants.printPose3d(targetPose);
    }

    public static void printPoseTagReef(Alliance alliance, String reefFace){
        int tagId = FieldConstants.getTagFromReef(alliance, reefFace);
        Pose3d reefTagPose = FieldConstants.getTagPose(tagId);
        System.out.print("\nREEF POSE FOR APRILTAG#" + tagId +":");
        FieldConstants.printPose3d(reefTagPose);
    }

    public static void printPoseTagCoralStation(Alliance alliance, String stationDirection){
        int csTagId = FieldConstants.getTagFromCoralStation(alliance, stationDirection);
        Pose3d csTagPose = FieldConstants.getTagPose(csTagId);
        System.out.print("\nCORAL STATION POSE AT " + csTagId + " " + alliance + " "+ stationDirection+":");
        FieldConstants.printPose3d(csTagPose);
    }

    public static void printPoseRobotToCoralStation(Alliance alliance, String stationDirection){
        Pose3d coralStationPose = FieldConstants.getRobotPoseToCoralStation(alliance, stationDirection);
        System.out.print("\nROBOT POSE TO CS " + alliance + " "+ stationDirection+":\t");
        FieldConstants.printPose3d(coralStationPose);
    }

    public static void printPoseRobotToTag(Alliance alliance, String reefFace){
        int tagId = FieldConstants.getTagFromReef(alliance, reefFace);
        Pose3d robotPose = FieldConstants.getRobotPoseToTag(tagId);
        System.out.print("\nROBOT POSE FACING TAG #" + tagId + " (" + alliance + "):\t");
        FieldConstants.printPose3d(robotPose);
    }




}