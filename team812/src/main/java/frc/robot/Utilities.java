/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.PoseEstimatorSubsystem;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Add your docs here.
 */
public class Utilities {
    
    private static boolean m_isBlueAlliance = false;
    private static boolean m_isRedAlliance = false;
    private static boolean m_isAutonomous = true;

    public static double scaleDouble(final double input, final double to_min, final double to_max) {
            final double from_min = -1.0;
            final double from_max = 1.0;
        double x;
        double scaled_x = 0.0;
        if( to_max > to_min  && from_max > from_min )
        {
            x =  input;
            scaled_x = ((x - from_min) * (to_max - to_min)) / 
                        (to_max - to_min) +
                        to_min;
        }
        return scaled_x;    
    }

    public static void setAutonomous() {
        m_isAutonomous = DriverStation.isAutonomous();
    }

    public static boolean isAutonomous() {
        return m_isAutonomous;
    }

    public static void setAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            m_isBlueAlliance = (alliance.get() == Alliance.Blue);  // Remember which alliance we are in.
            m_isRedAlliance =  (alliance.get() == Alliance.Red);
        }
        SmartDashboard.putBoolean("BlueAlliance", m_isBlueAlliance);   
    }
    
    public static boolean isRedAlliance() {
        return m_isRedAlliance;
    }

    public static boolean isBlueAlliance() {
        return m_isBlueAlliance;
    }

    public static void toSmartDashboard(String label, Pose2d pose) {
        SmartDashboard.putString(label, String.format("(%4.2f,%4.2f) %2.0f", pose.getX(), pose.getY(), pose.getRotation().getDegrees()) );
    }

    public static Pose2d pose180(Pose2d pose) {
        Rotation2d rotate180 = new Rotation2d(Math.PI);
        Rotation2d newRotation = pose.getRotation().rotateBy(rotate180);
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    // Create a new pose with the same X,Y coordinates rotated by <radians>
    public static Pose2d rotatePose(Pose2d pose, double radians) {
        //toSmartDashboard("RP Start", pose);
        Rotation2d rotation = new Rotation2d(radians);
        Rotation2d newRotation = pose.getRotation().rotateBy(rotation);
        //toSmartDashboard("RP End",new Pose2d(pose.getX(), pose.getY(), newRotation));
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    // Create a new pose with the same X,Y coordinates and the specified rotation
    public static Pose2d setPoseAngle(Pose2d pose, double radians) {
        //toSmartDashboard("SP Start", pose);
        Rotation2d newRotation = new Rotation2d(radians);
        //toSmartDashboard("SP End",new Pose2d(pose.getX(), pose.getY(), newRotation));
        return new Pose2d(pose.getX(), pose.getY(), newRotation);
    }

    public static Pose2d backToPose(Pose2d pose, double distance) {
        Translation2d rotatedDistance = new Translation2d(distance, 0).rotateBy(pose.getRotation());
        return new Pose2d(pose.getX() + rotatedDistance.getX(), pose.getY() + rotatedDistance.getY(), pose.getRotation());
    }

    public static Pose2d facingNearPose(Pose2d pose, double distance) {
        Rotation2d rotate180 = new Rotation2d(Math.PI);
        Translation2d rotatedDistance = new Translation2d(distance, 0).rotateBy(pose.getRotation());
        Rotation2d newRotation = pose.getRotation().rotateBy(rotate180);
        return new Pose2d(pose.getX() + rotatedDistance.getX(), pose.getY() + rotatedDistance.getY(), newRotation);
    }
    // Return direction of turn for angle a->b->c
    // -1 if counter-clockwise
    //  0 if collinear
    //  1 if clockwise
    public static int ccw(Translation2d a, Translation2d b, Translation2d c) {
        double area2 = (b.getX() - a.getX()) * (c.getY() - a.getY()) - (c.getX() - a.getX()) * (b.getY() - a.getY());
        if      (area2 < 0) return -1;
        else if (area2 > 0) return +1;
        else                return  0;
    }

    // Use a winding algorithm to determine if the point is in the polugon defined by
    // the array of points.  The polygon must be closed meaning that the last point
    // in the array should be the same as the first point in the array.
    public static boolean pointInPolygon(Translation2d [] polygon, Translation2d point) {
        int winding = 0;
        for (int i = 0; i < polygon.length-1; i++) {
            int ccw = ccw(polygon[i], polygon[i+1], point);
            if (polygon[i+1].getY() >  point.getY() && point.getY() >= polygon[i].getY())  // upward crossing
                if (ccw == +1) winding++;
            if (polygon[i+1].getY() <= point.getY() && point.getY() <  polygon[i].getY())  // downward crossing
                if (ccw == -1) winding--;
        }
        return (winding != 0);
    }
    
    /**
     * refineYCoordinate - use the ultrasonic sensor to set the Pose estimator's
     * Y axis position on the field.  For this to make sense, the robot has to be
     * rotated to -90 degrees (ie back facing the amp wall) and near the 
     * amp wall.
    */
    public static void refineYCoordinate() {
        boolean reset = false;
        double ultrasonicRange = RobotContainer.m_PingResponseUltrasonicSubsystem.getRange();
        if (ultrasonicRange < 2.0 /* Meters */) {
            Pose2d currentPose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();
            if (Math.abs(currentPose.getRotation().getDegrees() -  -90.0) < 5.0 /* degrees */) {
                reset = true;
                RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(
                new Pose2d(
                    currentPose.getX(),
                    FieldConstants.yMax - ultrasonicRange - DriveConstants.kBackToCenterDistance,
                    currentPose.getRotation()
                )
                );
            }
            
        }
        // For debug, display whether we did reset the coordinate or not.
        SmartDashboard.putBoolean("refineY", reset);
    }

    public static void allianceSetCurrentPose(Pose2d newPose) {
        if (isBlueAlliance()) {
            RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(newPose);
         } else { 
            // For the red alliance translate the X coordinates and mirror the rotation
            double x = FieldConstants.xMax - newPose.getX();
            double rotation = MathUtil.inputModulus(
                Math.PI - newPose.getRotation().getRadians(),
                -Math.PI,
                Math.PI
            );
            RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(new Pose2d(x, newPose.getY(), new Rotation2d(rotation)));
        }
    }
    
    public static Pose2d getAllianceRobotAmpPose(PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Pose2d robotPose = null;
        if (isBlueAlliance()) {
            Pose2d tag = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.BLUE_AMP.id());
            // This should position the robot back to the AMP touching the wall.
            robotPose = new Pose2d(tag.getX(), tag.getY() - DriveConstants.kBackToCenterDistance, tag.getRotation());
    
        } else if (isRedAlliance()) {
            Pose2d tag = poseEstimatorSubsystem.getAprilTagPose(VisionConstants.AprilTag.RED_AMP.id());
            // This should position the robot back to the AMP touching the wall.
            robotPose = new Pose2d(tag.getX(), tag.getY() - DriveConstants.kBackToCenterDistance, tag.getRotation());
        }
        else {
            robotPose = null; // Hack:: if we dont know the alliance. Dont move. 
        }
        
        return robotPose;
    }

    /**
     * resetPoseAtAmp - set the pose estimator's pose to be at the alliance amp.
     * This is to help keep the pose estimator's result up to date as reading april
     * tags it not working well when the robot is moving quickly and due to 
     * the steep angle of the camera limiting the location on the field where
     * the robot can see an april tag.
    */
    public static void resetPoseAtAmp() {
        boolean reset = false;
        double ultrasonicRange = RobotContainer.m_PingResponseUltrasonicSubsystem.getRange();
        // Verify we are up against the wall.
        if (ultrasonicRange < 0.030 /* Meters */) { // Ie 3 centimeters
            // Trusting the gyro most so dont reset rotation instead, use the current pose's rotation.
            Pose2d currentPose = RobotContainer.m_PoseEstimatorSubsystem.getCurrentPose();
            if (Math.abs(currentPose.getRotation().getDegrees() -  -90.0) < 5.0 /* degrees */) {
                Pose2d robotPoseAtAmp = getAllianceRobotAmpPose(RobotContainer.m_PoseEstimatorSubsystem);
                // If the alliance is unknown, this cant be used.
                if (robotPoseAtAmp != null) {
                    reset = true;
                    // It seems safe, set the pose to be the proper pose for the robot
                    // perfectly positioned in front of the amp, touching the wall.
                    RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(
                    new Pose2d(
                        robotPoseAtAmp.getTranslation(),
                        currentPose.getRotation()
                    )
                    );
                }
            }
            
        }
        // For debug, display whether we did reset the coordinate or not.
        SmartDashboard.putBoolean("PoseReset", reset);
    }
    
    // Boolean Supplier to return true if we are within 45 seconds of the end of the match.
    public static BooleanSupplier endGame = ()->DriverStation.getMatchTime() >= 2.5*60.0 - 45.0;
    

}