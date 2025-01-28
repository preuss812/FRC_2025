// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;

/** 
 * This class supplies data to create trajectories from any point on the field to the Processor 
 * The hope is that these will be useful for autonomous mode and possibly (but unlikey) for enable semi-automatic driving during teleop.
 * The strategy is the divide the field into approximately 2 meter squares and then create plans based on the
 * best path to travel from each starting square.
 */
public class TrajectoryPlans {

    public static int numXSquares = 8;
    public static int numYSquares = 4;
    public static double dx = FieldConstants.xMax/numXSquares;
    public static double dy = FieldConstants.yMax/numYSquares;
    public static ArrayList<SequentialCommandGroup> autoPlans = new ArrayList<SequentialCommandGroup>();
    public static ArrayList<Trajectory> autoPaths = new ArrayList<Trajectory>();
    public static ArrayList<String>     autoNames = new ArrayList<String>();
    public static ArrayList<Pose2d[]>   waypoints = new ArrayList<Pose2d[]>();
    public static final TrajectoryConfig m_defaultTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond/5.0,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared/5.0)
        .setKinematics(DriveConstants.kDriveKinematics);

    // Define a gridded map of the field to define a path from each square to the blue alliance processor.
    // Currently these paths are crude and need some refinement if they are to be used during a match.
    public static final Pose2d[][] BlueProcessorPlan = new Pose2d[][]
    {
        // column 0
        {
            fieldSquareToPose(1,0),
            fieldSquareToPose(1,0),
            fieldSquareToPose(0,1),
            fieldSquareToPose(0,2)
        },
        // column 1
        {
            fieldSquareToPose(2,0),
            fieldSquareToPose(2,0),
            fieldSquareToPose(0,1),
            fieldSquareToPose(0,2),
        },
        // column 2
        {
            fieldSquareToPose(2,0),
            fieldSquareToPose(2,0),
            fieldSquareToPose(3,2),
            fieldSquareToPose(3,2)
        },
        // column 3
        {
            // column 0
            fieldSquareToPose(2,0),
            fieldSquareToPose(3,0),
            fieldSquareToPose(3,1),
            fieldSquareToPose(3,2)
        },
        // column 4
        {
            fieldSquareToPose(4,1),
            fieldSquareToPose(4,2),
            fieldSquareToPose(3,2),
            fieldSquareToPose(3,3)
        },
        // column 5
        {
            fieldSquareToPose(4,1),
            fieldSquareToPose(4,2),
            fieldSquareToPose(4,2),
            fieldSquareToPose(3,2)
        },
        // column 6
        {
            fieldSquareToPose(5,0),
            fieldSquareToPose(7,2),
            fieldSquareToPose(5,3),
            fieldSquareToPose(5,3)
        },  
        // column 7
        {
            fieldSquareToPose(7,1),
            fieldSquareToPose(7,2),
            fieldSquareToPose(6,3),
            fieldSquareToPose(6,3)
        }   
    };

    // Create a red alliance version of the plans by transforming the blue alliance plans
    public static final Pose2d[][] RedProcessorPlan = transformPlan(BlueProcessorPlan, FieldConstants.blueToRedTransform);

    /**
     * Construct a Trajectory plans object.  Currently this does nothing as the whole class is static.
    */
    public TrajectoryPlans() {
    }

    /**
     * Convert an array of Pose2d objects (=a plan) using the supplied transform
     * @param trajectoryPlan - The Pose2d objects to be transformed.
     * @param transformPose - The Transform2d to be used.
     * @return Pose2d[][] containing the transformed Pose2d objects.
     */
    public static Pose2d[][] transformPlan(Pose2d[][] trajectoryPlan, Pose2d transformPose) {
        Pose2d[][] newPlan = new Pose2d[numXSquares][numYSquares];
        for (int i = 0; i < numXSquares; i++) {
            for (int j = 0; j < numYSquares; j++) {
                Pose2d sourcePose = trajectoryPlan[numXSquares-i-1][numYSquares-j-1];
                Pose2d transformedPose = sourcePose.relativeTo(transformPose);
                newPlan[i][j] = transformedPose;
            }
        }
        return newPlan;
    }
    
    /**
     * Create a list of poses from the startingPose to the end of the path using the trajectory plan grid as a map.
     * @param trajectoryPlan - the map from each field square to some destination (e.g. the Processsor)
     * @param startingPose - a location on the field used as a starting point for the plan.
     * @return - a list of Pose2d object to get from the startingPose to the destination.
     */
    public static List<Pose2d> planTrajectory( Pose2d[][] trajectoryPlan, Pose2d startingPose) {
        List<Pose2d> list = new ArrayList<>();
        // convert the coordinates into indexes for 2 by 2 meter squares.
        // 0,0 is the lower left of the field by the Red Alliance source.
        int[] ij = poseToFieldSquare(startingPose);
        int i = ij[0];
        int j = ij[1];
        int n = 0;
        boolean done = false;
        //list.add(startingPose);
        while (!done) {
            Pose2d nextPose = trajectoryPlan[i][j];
            int [] nextid = poseToFieldSquare(nextPose);
            //if we stay in the same square, we are done.
            if (nextid[0] == i && nextid[1] == j) {
                done = true;
            } else {
                list.add(nextPose);
                n++;
                i = nextid[0];
                j = nextid[1];
                //if (n > 8) 
                //    done = true;
            }
        }
        return list;
    }

    /**
     * return the (i,j) grid position on the field from a location on the field.
     * @param fieldLocation - a Pose2d with the field location
     * @return - an int[2] array with the (i,j) field square for that location.
     */
    public static int[] poseToFieldSquare(Pose2d fieldLocation) {
        int[] square = new int[2];
        square[0] = (int)(fieldLocation.getX()/dx);
        square[1] = (int)(fieldLocation.getY()/dy);
        return square;
    }
    
    /**
     * return a Pose2d object for a field square representing where in that square the robot should drive.
     * @param i - (int) the i-axis location of the field square.
     * @param j - (int) the j-axis location of the field square.
     * @return - a Pose2d for driving to/through that field square.  Currently the center of that square.
     */
    public static Pose2d fieldSquareToPose(int i, int j) {
        return new Pose2d(dx*(i+0.5),dy*(j+0.5), new Rotation2d(0));
        
    }

    /**
     * create a Trajectory using the driveTrain and config using the list of waypoints.
     * @param driveTrain - the robot's driveTrain subsystem.
     * @param waypoints - an array of Pose2d objects describing the starting location, waypoints and ending location for the driving.
     * @param config - a TrajectoryConfig describing the driving parameters (eg max speed, acceleration, etc).
     * @return - a Trajectory object with smoothed (spline) path for the driving path or null if no path could be calculated.
     */
    private static Trajectory createTrajectory(DriveSubsystemSRX driveTrain, Pose2d[] waypoints, TrajectoryConfig config) {
        Trajectory trajectory = null;
        Pose2d startingPose = waypoints[0];
        Pose2d endingPose = waypoints[waypoints.length - 1];

        List<Translation2d>  translationWaypoints = new ArrayList<Translation2d>();
        try {
            //Pose2d fakeTargetPose = new Pose2d(endingPose.getTranslation(), new Rotation2d(0));
            for (int i = 1; i < waypoints.length - 1; i++) {
               translationWaypoints.add(waypoints[i].getTranslation());
            }
            trajectory = TrajectoryGenerator.generateTrajectory(          
                startingPose,  // We are starting where we are.
                // Pass through these zero interior waypoints, this should probably be something to make sure we dont crash into other robots.
                translationWaypoints,
                endingPose,
                config != null ? config : m_defaultTrajectoryConfig
            ); // use default config is none was specified.
            // Next line is debug to display the path on the shuffleboard.Vision.field
            RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory);
        }
        catch (Exception e) {
            // Let null return as the trajectory and the caller must handle it.
        }
        return trajectory;
    }
    
    /**
     * Create a SwerveControllerCommand to drive following a path
     * @param driveTrain - the robot's drive train subsystem.
     * @param poseEstimatorSubsystem - the robot's pose estimator subsystem.
     * @param waypoints - an array of Pose2d the is the path to be driven.
     * @param config - driving parameters (e.g. max speed, max acceleration, etc )
     * @return - (SwerveControllerCommand) command to run that will drive the robot following the path (or null if no path found)
     */
    public static SwerveControllerCommand swerveControllerCommandFactory(DriveSubsystemSRX driveTrain, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d[] waypoints, TrajectoryConfig config) {
        SwerveControllerCommand command = null;
        // The SwerveControllerCommand needs a holonomic set of pid controllers.
        // Create the rotatation controller here and the x,y controllers in-line below.
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // angles will be converted to be within the range +/- pi. (e.g. 361 degrees will be converted to 1 degree)

        // Calculate the smoothed path through the waypoints.
        Trajectory trajectory = createTrajectory(driveTrain, waypoints, config);

        // If we got a path, create the driving command.
        if (trajectory != null) {
            command = new SwerveControllerCommand(
            trajectory,
            poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);
        }
        return command;
    }

    /**
     * Create predefined autonomous routines for use during the autonomous period.
     * For now, 6 routines are defined.  One for each april tag on the reef.
     * Currently, this only supports blue alliance.
     * It needs to be enhanced to also or alternatively create proper paths for the red
     * alliance by transforming the routines using the DriveTrain.blueToRedPose() method
     * if the robot is in the red alliance.
     */
    public static void buildAutoTrajectories() {
        // The staring poses will be on the blue starting line facing back toward the blue drive station
        Rotation2d startingRotation = new Rotation2d(Math.PI);

        // Get/create poses for each Reef April tag and barge april tag
        // for omre concise coding below.
        Pose2d AT14 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(14);
        Pose2d AT15 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(15);
        Pose2d AT17 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(17);
        Pose2d AT18 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(18);
        Pose2d AT19 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(19);
        Pose2d AT20 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(20);
        Pose2d AT21 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(21);
        Pose2d AT22 = RobotContainer.m_PoseEstimatorSubsystem.getAprilTagPose(22);
        Pose2d nearAT17 = DriveConstants.robotFrontAtPose(AT17);
        Pose2d nearAT18 = DriveConstants.robotFrontAtPose(AT18);
        Pose2d nearAT19 = DriveConstants.robotFrontAtPose(AT19);
        Pose2d nearAT20 = DriveConstants.robotFrontAtPose(AT20);
        Pose2d nearAT21 = DriveConstants.robotFrontAtPose(AT21);
        Pose2d nearAT22 = DriveConstants.robotFrontAtPose(AT22);

        // Build a path adding it to the autoChooser which will select the autonomous routine
        autoNames.add("My Barge to Opposite");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size());
        waypoints.add(new Pose2d[] {
            new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
            new Pose2d(FieldConstants.zeroToReef,AT14.getY(), startingRotation),
            new Pose2d(FieldConstants.zeroToReef*0.66,AT18.getY(), startingRotation),
            nearAT18
        });
        autoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints.get(waypoints.size()-1),
            m_defaultTrajectoryConfig
        ));

        // Build a path adding it to the autoChooser which will select the autonomous routine
        autoNames.add("My Barge to Far Side");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size());
        waypoints.add(new Pose2d[] {
            new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
            new Pose2d(AT19.getX(),AT14.getY(), startingRotation),
            nearAT19
        });
        autoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints.get(waypoints.size()-1),
            m_defaultTrajectoryConfig
        ));

        autoNames.add("My Barge to Near Side");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size());
        waypoints.add(new Pose2d[] {
            new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
            new Pose2d((AT20.getX()+FieldConstants.blueStartLine)/2.0,AT14.getY(), startingRotation),
            nearAT20
        });
        autoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints.get(waypoints.size()-1),
            m_defaultTrajectoryConfig
        ));

        // Build a path adding it to the autoChooser which will select the autonomous routine
        autoNames.add("Center Straight");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size());
        waypoints.add(new Pose2d[] {
            new Pose2d(FieldConstants.blueStartLine,FieldConstants.yCenter, startingRotation),
            new Pose2d((AT21.getX()+FieldConstants.blueStartLine)/2.0,FieldConstants.yCenter, startingRotation),
            nearAT21
        });
        autoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints.get(waypoints.size()-1),
            m_defaultTrajectoryConfig
        ));

        // Build a path adding it to the autoChooser which will select the autonomous routine
        autoNames.add("Their Barge to Near Side");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size());
        waypoints.add(new Pose2d[] {
            new Pose2d(FieldConstants.blueStartLine,AT15.getY(), startingRotation),
            new Pose2d((AT22.getX()+FieldConstants.blueStartLine)/2.0,AT15.getY(), startingRotation),
            nearAT22
        });
        autoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints.get(waypoints.size()-1),
            m_defaultTrajectoryConfig
        ));

        // Build a path adding it to the autoChooser which will select the autonomous routine
        autoNames.add("Their Barge to Far Side");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size());
        waypoints.add(new Pose2d[] {
            new Pose2d(FieldConstants.blueStartLine,AT15.getY(), startingRotation),
            new Pose2d((AT17.getX()+FieldConstants.blueStartLine)/2.0,AT15.getY(), startingRotation),
            nearAT17
        });
        autoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints.get(waypoints.size()-1),
            m_defaultTrajectoryConfig
        ));
        SmartDashboard.putData("AutoSelector", Robot.autoChooser);
    }

    /**
     * Returns a sequential command group that displays and executes an autonomous routine.
     * This should be enhanced to score the coral and possible grab an algae and score it in
     * the processor. 
     * @param driveTrain                - The drive train
     * @param poseEstimatorSubsystem    - the pose estimator
     * @param waypoints                 - an array of poses for the start, waypoints and end of the driving path.
     * @param config                    - drive train parameters to control robot speeds
     * @return
     */
    public static SequentialCommandGroup debugAutoCommand(DriveSubsystemSRX driveTrain, PoseEstimatorSubsystem poseEstimatorSubsystem, Pose2d[] waypoints, TrajectoryConfig config) {
        SwerveControllerCommand command = null;
        
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);


        Trajectory trajectory = createTrajectory(driveTrain, waypoints, config);
        if (trajectory != null) {
            command = new SwerveControllerCommand(
            trajectory,
            poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            driveTrain::setModuleStates,
            driveTrain);
        }
        return new SequentialCommandGroup(
            // might need some robot initialization here (e.g. home arm, check to see an april tag to make sure the robot is where it is assumed to be)
            new InstantCommand( () -> RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(waypoints[0])),
            new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.setRobotPose(waypoints[0])), // for debug
            new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory)), // for debug
            command,
            new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.setRobotPose(waypoints[waypoints.length-1])) // for debug
            // may need a driveToPose to perfectly position the robot.
            // will need some arm motion to socre the coral.
            // could add additional actions to grab an algea and drive to the processor and score there.
        );
    }

    /**
     * This method checks our assumptions that the field is (almost) perfectly rotated around the center of the field.
     * Since we are transforming blue trajectories and locations to make red trajectories and locations, we need
     * the transformed coordinates to be correct.
     * @return - true if the assumptions are met, otherwise false.
     */
    public static boolean checkAprilTagMirroring(PoseEstimatorSubsystem poseEstimatorSubsystem) {
        double translationThreshold = 0.002; // 2mm
        double rotationThreshold = 0.001; // 0.001 Radians which is about 1/20th of a degree
        if (Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(1),poseEstimatorSubsystem.getAprilTagPose(13),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(2),poseEstimatorSubsystem.getAprilTagPose(12),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(3),poseEstimatorSubsystem.getAprilTagPose(16),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(4),poseEstimatorSubsystem.getAprilTagPose(15),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(5),poseEstimatorSubsystem.getAprilTagPose(14),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(6),poseEstimatorSubsystem.getAprilTagPose(19),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(7),poseEstimatorSubsystem.getAprilTagPose(18),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(8),poseEstimatorSubsystem.getAprilTagPose(17),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(9),poseEstimatorSubsystem.getAprilTagPose(22),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(10),poseEstimatorSubsystem.getAprilTagPose(21),translationThreshold, rotationThreshold)
        &&  Utilities.comparePoses(poseEstimatorSubsystem.getAprilTagPose(11),poseEstimatorSubsystem.getAprilTagPose(20),translationThreshold, rotationThreshold)) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Check that all the trajectory paths generated do not cause a crash.
     * This may throw a null pointer exception.
     */
    public static void checkAllTrajectories() throws NullPointerException {
        // Try all possible plans to makesure there are now obvious bad moves in the plans.
        for (int i = 1; i < 8; i++) {
            for (int j = 1; j < 4; j++) {
            SmartDashboard.putNumber("I", i);
            SmartDashboard.putNumber("J", j);
            
            List<Pose2d> blueProcessorPlan = TrajectoryPlans.planTrajectory(
                TrajectoryPlans.BlueProcessorPlan, TrajectoryPlans.fieldSquareToPose(i, j)
            );
            }
        }
        // Try all possible plans to makesure there are now obvious bad moves in the plans.
        for (int i = 1; i < 8; i+= 1) {
            for (int j = 1; j < 4; j+= 1) {
            SmartDashboard.putNumber("I", i);
            SmartDashboard.putNumber("J", j);
            List<Pose2d> redProcessorPlan = TrajectoryPlans.planTrajectory(
                TrajectoryPlans.RedProcessorPlan, TrajectoryPlans.fieldSquareToPose(i, j)
            );
            }
        }
    }
}
