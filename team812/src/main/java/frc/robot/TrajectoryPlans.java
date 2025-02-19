// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.Util;
import com.revrobotics.servohub.ServoHubLowLevel.PeriodicStatus0;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.PreussSwerveControllerCommand;
import frc.robot.commands.RotateRobotCommand;
import frc.robot.commands.VerifyExpectedAprilTagCommand;
import frc.robot.commands.VerifyStartingPositionCommand;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX.DrivingMode;

/** 
 * This class supplies data to create trajectories from any point on the field to the Processor 
 * The hope is that these will be useful for autonomous mode and possibly (but unlikey) for enable semi-automatic driving during teleop.
 * The strategy is the divide the field into approximately 2 meter squares and then create plans based on the
 * best path to travel from each starting square.
 */
public class TrajectoryPlans {

    private static boolean debug = true;
    public static int numXSquares = 8;
    public static int numYSquares = 4;
    public static double dx = FieldConstants.xMax/numXSquares;
    public static double dy = FieldConstants.yMax/numYSquares;
    public static ArrayList<SequentialCommandGroup> blueAutoPlans = new ArrayList<SequentialCommandGroup>();
    public static ArrayList<SequentialCommandGroup> redAutoPlans = new ArrayList<SequentialCommandGroup>();
    public static ArrayList<Trajectory> autoPaths = new ArrayList<Trajectory>();
    public static ArrayList<String>     autoNames = new ArrayList<String>();
    public static ArrayList<Pose2d[]>   waypoints = new ArrayList<Pose2d[]>();
    public static ArrayList<Integer> expectedAprilTag = new ArrayList<Integer>();

    // For now, default speeds are the debug/slow speeds.
    public static final TrajectoryConfig m_debugTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond/2.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared/2.5)
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);
    public static final TrajectoryConfig m_fullSpeedTrajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true);
        ;
        public static final TrajectoryConfig m_defaultTrajectoryConfig = m_debugTrajectoryConfig;

    // Define a gridded map of the field to define a path from each square to the blue alliance processor.
    // Currently these paths are crude and need some refinement if they are to be used during a match.
    public static final Pose2d[][] BlueProcessorPlan = new Pose2d[][]
    {
        // column 0
        {
            fieldSquareToPose(1,0),
            fieldSquareToPose(2,0),
            fieldSquareToPose(1,1),
            fieldSquareToPose(0,2)
        },
        // column 1
        {
            fieldSquareToPose(1,0),
            fieldSquareToPose(2,0),
            fieldSquareToPose(0,2),
            fieldSquareToPose(0,2),
        },
        // column 2
        {
            fieldSquareToPose(2,0),
            fieldSquareToPose(2,1),
            fieldSquareToPose(3,2),
            fieldSquareToPose(3,2)
        },
        // column 3
        {
            fieldSquareToPose(3,0),
            fieldSquareToPose(3,1),
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
            fieldSquareToPose(5,2),
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
        list.add(startingPose);
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
        square[0] = MathUtil.clamp((int)(fieldLocation.getX()/dx), 0, numXSquares-1);
        square[1] = MathUtil.clamp((int)(fieldLocation.getY()/dy), 0, numYSquares-1);
        return square;
    }
    
    /**
     * return a Pose2d object for a field square representing where in that square the robot should drive.
     * @param i - (int) the i-axis location of the field square.
     * @param j - (int) the j-axis location of the field square.
     * @return - a Pose2d for driving to/through that field square.  Currently the center of that square.
     */
    public static Pose2d fieldSquareToPose(int i, int j) {
        double x = dx*(i+0.5);
        double y = dy*(j+0.5);

        if (i == 0) {
            x += 0.3*dx;
        } else if (i == 1) {
            x -= 0.4*dx;
        } else if (i == 3) {
            x -= 0.3*dx;
        }
        if (j == 1) {
            y -= 0.4 *dy;
        } else if (j == 2) {
            y += 0.2 * dy;
        } else if (j == 3) {
            y -= 0.1*dy;
        }
        
        return new Pose2d(x,y, new Rotation2d(0));
        
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
        boolean useListOfPoses = true;

        try {

            if (useListOfPoses) {
                // Try the version of generateTrajectory that takes a list of poses instead of a list of translations.
                // The hope is that this generates better robot rotational control - dph 2025-01-28
                List<Pose2d> waypointList = Arrays.asList(waypoints);
                trajectory = TrajectoryGenerator.generateTrajectory( waypointList, config != null ? config : m_defaultTrajectoryConfig);
            } else {
                // Generate a trajectory with a starting and ending pose and a list of translation waypoints.
                List<Translation2d>  translationWaypoints = new ArrayList<Translation2d>();
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
            }
            if (debug)
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
     * 
     * @param name       - the display name for this autonomous mode.
     * @param waypoints  - an array of poses to be used to generate the driving path
     * @param config     - the trajectory configuration to be used to generate the driving path.
     * @param aprilTagID - the apriltag expected to be seen when the driving begins.
     *                     This is used to verify the starting position and possibly terminate the
     *                     autonomous mode if the robot is not where it is expected to be.
     *                     For now, the checking is done in the VerifyStartingPositionCommand which
     *                     is added to a sequential command group which would prevent this command from
     *                     being executed.
     * 
     * Note: the grouping of commands here and in Autonomous.java is not ideal and should be refactored.
     */
    public static void addAutoMode(
        String name,
        Pose2d[] waypoints,
        TrajectoryConfig config,
        int aprilTagID) {

        autoNames.add(name);
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        blueAutoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints,
            config,
            false
        ));
        redAutoPlans.add(debugAutoCommand(
            RobotContainer.m_robotDrive,
            RobotContainer.m_PoseEstimatorSubsystem,
            waypoints,
            config,
            true
        ));
        expectedAprilTag.add(aprilTagID);
    }

    /**
     * Pose facing reef, create a pose that rotates the robot so the camera faces the center of the reef.
     * @param x - a double that is the x field coordinate.
     * @param y = a double that is the y field coordinate.
     * @return - a Pose2d object using the supplied x and y and a rotation that will rotate the robot to face the reef.
     */
    public static Pose2d poseWithCameraFacingTheReef(double x, double y) {
        return new Pose2d(x, y, new Rotation2d(FieldConstants.robotHeadingForCameraToBlueReefCenter(x,y)));
    }
    /**
     * Create predefined autonomous routines for use during the autonomous period.
     * For now, 6 routines are defined.  One for each april tag on the reef.
     * It creates paths for both blue and red alliances.
     * Both paths are generated at startup so that it will be certain that the
     * required path (red or blue) can be selected at autonomousInit when the alliance
     * is sure to be known.
     * 
     * Note: Path numbering is problematic so be careful when adding new options to keep the
     * numbering consistent.
     * 
     */
    public static void buildAutoTrajectories() {
        // The staring poses will be on the blue starting line facing back toward the blue drive station
        Rotation2d startingRotation = new Rotation2d(0.0);
        double offsetFromAprilTag = Units.inchesToMeters(-5.0); // 0.5 meters from the april tag
        if (debug) checkAllTrajectories();
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

        Pose2d nearAT17 = DriveConstants.robotRearAtPose(AT17, offsetFromAprilTag);
        Pose2d nearAT18 = DriveConstants.robotRearAtPose(AT18, offsetFromAprilTag);
        Pose2d nearAT19 = DriveConstants.robotRearAtPose(AT19, offsetFromAprilTag);
        Pose2d nearAT20 = DriveConstants.robotRearAtPose(AT20, offsetFromAprilTag);
        Pose2d nearAT21 = DriveConstants.robotRearAtPose(AT21, offsetFromAprilTag);
        Pose2d nearAT22 = DriveConstants.robotRearAtPose(AT22, offsetFromAprilTag);
        //TrajectoryConfig config = m_debugTrajectoryConfig;
        TrajectoryConfig config = m_defaultTrajectoryConfig;

        // Add the defualt plan which is not yet defined, for now do nothing.
        autoNames.add("Robot Makes the Plan");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        waypoints.add(new Pose2d[]{}); // Empty array.
        blueAutoPlans.add(new SequentialCommandGroup(Commands.none()) ); // TODO; Handle here or in getAutonomousCommand
        redAutoPlans.add(new SequentialCommandGroup(Commands.none()) ); // TODO; Handle here or in getAutonomousCommand
        expectedAprilTag.add(0);

         // Add the defualt plan which is not yet defined, for now do nothing.
         autoNames.add("Drive off Line and Stop");
         Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
         waypoints.add(new Pose2d[]{}); // Empty array.
         blueAutoPlans.add(new SequentialCommandGroup(Commands.none()) );  // TODO: add drive command
         redAutoPlans.add(new SequentialCommandGroup(Commands.none()) );  // TODO: add drive command
         expectedAprilTag.add(0);

          // Add the defualt plan which is not yet defined, for now do nothing.
        autoNames.add("Do Nothing");
        Robot.autoChooser.addOption(autoNames.get(autoNames.size()-1), autoNames.size()-1);
        waypoints.add(new Pose2d[]{}); // Empty array.
        blueAutoPlans.add(new SequentialCommandGroup(Commands.none()) );
        redAutoPlans.add(new SequentialCommandGroup(Commands.none()) );
        expectedAprilTag.add(0);

        // Build a path adding it to the autoChooser which will select the autonomous routine
        addAutoMode(
            "My Barge to Opposite"
            ,new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
                //new Pose2d(AT20.getX(),AT14.getY(), startingRotation),
                //new Pose2d(AT19.getX(),AT14.getY(), startingRotation),
                new Pose2d(FieldConstants.zeroToReef,AT14.getY(), startingRotation),
                new Pose2d(FieldConstants.zeroToReef*0.66,AT18.getY()+1.0, new Rotation2d(Math.PI*0.5)),
                nearAT18
            },
            config,
            20);

        // Build a path adding it to the autoChooser which will select the autonomous routine
        addAutoMode("My Barge to Far Side"
            ,new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
                new Pose2d(AT19.getX(),AT14.getY(), startingRotation),
                nearAT19           
            }
            ,config
            ,20
        );

        addAutoMode(
            "My Barge to Near Side"
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT14.getY(), startingRotation),
                new Pose2d((AT20.getX()+FieldConstants.blueStartLine)/2.0,AT14.getY(), startingRotation),
                //poseWithCameraFacingTheReef((AT20.getX()+FieldConstants.blueStartLine)/2.0,AT14.getY()),  Adds extra squiggles to the path
                nearAT20
            }
            , config
            , 20
        );
        
        // Build a path adding it to the autoChooser which will select the autonomous routine
        addAutoMode(
            "Center Straight"
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine ,FieldConstants.yCenter, startingRotation),
                new Pose2d((AT21.getX()+FieldConstants.blueStartLine)/2.0,FieldConstants.yCenter, startingRotation),
                nearAT21
            }
            , config
            , 21
            );

        // Build a path adding it to the autoChooser which will select the autonomous routine
        addAutoMode(
            "Their Barge to Near Side"  
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT15.getY(), startingRotation),
                new Pose2d((AT22.getX()+FieldConstants.blueStartLine)/2.0,AT15.getY(), startingRotation),
            nearAT22
            }
            , config
            , 22
        );

        // Build a path adding it to the autoChooser which will select the autonomous routine
        addAutoMode("Their Barge to Far Side"
            , new Pose2d[] {
                new Pose2d(FieldConstants.blueStartLine,AT15.getY(), startingRotation),
                new Pose2d(AT22.getX(),AT15.getY(), startingRotation),
                new Pose2d(AT17.getX(),AT15.getY(), startingRotation),
                nearAT17
            }
            , config
            , 22
        );

        SmartDashboard.putData("AutoSelector", Robot.autoChooser);
    }

    public static Command getSwerveCommand(
        DriveSubsystemSRX driveTrain
        ,PoseEstimatorSubsystem poseEstimatorSubsystem
        ,Trajectory trajectory
     ) {
        Command command;
        if (trajectory != null) {
            command = new PreussSwerveControllerCommand(
            trajectory,
            poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
            driveTrain::setModuleStates,
            driveTrain);
        } else {
            command = Commands.none();
        }
        return command;
    }

    /**
     * Returns a rotation for the robot to face the reef.  Used as a lambda function to supply the rotation to the SwerveControllerCommand.
     * @param poseEstimatorSubsystem
     * @return - the rotation in radians for the robot to face the reef in field coordinates.
     */
    public static Rotation2d reefFacingRotationSupplier(PoseEstimatorSubsystem poseEstimatorSubsystem) {
        Rotation2d rotation = new Rotation2d(
                Autonomous.robotHeadingForCameraToReefCenter(
                    poseEstimatorSubsystem.getCurrentPose().getX()
                    , poseEstimatorSubsystem.getCurrentPose().getY()
                ) 
        );
        return rotation;
    } 

    public static Command getReefFacingSwerveCommand(
        DriveSubsystemSRX driveTrain
        ,PoseEstimatorSubsystem poseEstimatorSubsystem
        ,Trajectory trajectory
     ) {
        Command command;
        if (trajectory != null) {
            ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
            command = new PreussSwerveControllerCommand(
            trajectory,
            poseEstimatorSubsystem::getCurrentPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            () -> reefFacingRotationSupplier(poseEstimatorSubsystem),
            driveTrain::setModuleStates,
            driveTrain);
        } else {
            command = Commands.none();
        }
        return command;
    }
    
    /**
     * Returns a sequential command group that displays and executes an autonomous routine.
     * This should be enhanced to score the coral and possible grab an algae and score it in
     * the processor. 
     * @param driveTrain                - The drive train
     * @param poseEstimatorSubsystem    - the pose estimator
     * @param waypoints                 - an array of poses for the start, waypoints and end of the driving path.
     * @param config                    - drive train parameters to control robot speeds
     * @param convertToRed              - if true, the waypoints will be converted to red alliance waypoints.
     * @return
     */
    public static SequentialCommandGroup debugAutoCommand(
        DriveSubsystemSRX driveTrain
        , PoseEstimatorSubsystem poseEstimatorSubsystem
        , Pose2d[] blueWaypoints
        , TrajectoryConfig config
        , boolean convertToRed) {
        Command  command = null;
        Pose2d[] waypoints;
        Pose2d[] redWaypoints;
        
        boolean testing = true;
        // If we are the red alliance, we need to transform the waypoints to be red alliance waypoints.
        if (convertToRed) {
            redWaypoints = new Pose2d[blueWaypoints.length];
            for (int i = 0; i < blueWaypoints.length; i++) {
                redWaypoints[i] = FieldConstants.BlueToRedPose(blueWaypoints[i]);
            }
            waypoints = redWaypoints;
        } else {
            waypoints = blueWaypoints;
        }
        Trajectory trajectory = createTrajectory(driveTrain, waypoints, config);
        if (!testing) {
            if (trajectory != null) {
                command = getSwerveCommand(driveTrain, poseEstimatorSubsystem, trajectory);
            }
        } else {
            command = getReefFacingSwerveCommand(driveTrain, poseEstimatorSubsystem, trajectory);
        }

        return new SequentialCommandGroup(
            // might need some robot initialization here (e.g. home arm, check to see an april tag to make sure the robot is where it is assumed to be)
            new InstantCommand( () -> RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(waypoints[0])),
            new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.PRECISION)),
            //new RotateRobotCommand(RobotContainer.m_robotDrive, Units.degreesToRadians(30.0), true),
            //new VerifyExpectedAprilTagCommand(RobotContainer.m_PoseEstimatorSubsystem, FieldConstants.BlueAlliance, 20),
            //new InstantCommand( () -> RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(waypoints[0])),
           // new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.setRobotPose(waypoints[0])), // for debug
            new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory)), // for debug
            command
            //,new InstantCommand( () -> RobotContainer.m_PoseEstimatorSubsystem.setCurrentPose(waypoints[waypoints.length-1]))
            //new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.setRobotPose(waypoints[waypoints.length-1])) // for debug
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
        if (!debug) return true;    // Dont waste time if we are not debugging.
        boolean result = true; // Assume success.  Will reset if there is a mismatch.
        double translationThreshold = 0.002; // 2mm
        double rotationThreshold = 0.001; // 0.001 Radians which is about 1/20th of a degree
        for (int i = 1; i <= 22; i++) {
            if (!Utilities.comparePoses(
                poseEstimatorSubsystem.getAprilTagPose(i),
                poseEstimatorSubsystem.getAprilTagPose(FieldConstants.complementaryAprilTag[i]),
                translationThreshold,
                rotationThreshold
                )
            ) {
                result = false;
            }
        }
        /*
         
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
         */
         
        return result;
    }

    /**
     * Check that all the trajectory paths generated do not cause a crash.
     * This may throw a null pointer exception.
     */
    public static void checkAllTrajectories() throws NullPointerException {
        if (!debug) return;  // Dont waste time if we are not debugging.
        
        // Verify that the apriltag info matches our expectations
        SmartDashboard.putBoolean("AprilMirrorCheck", checkAprilTagMirroring(RobotContainer.m_PoseEstimatorSubsystem));
        
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

    public static SequentialCommandGroup robotDanceCommand(DriveSubsystemSRX driveTrain, PoseEstimatorSubsystem poseEstimatorSubsystem, TrajectoryConfig config) {
        Command command = null;
        
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Pose2d[] danceMoves = new Pose2d[] {
            new Pose2d(FieldConstants.blueRowOfAlgae,FieldConstants.yCenter,new Rotation2d(0.0)),
            new Pose2d(FieldConstants.blueRowOfAlgae,FieldConstants.yCenter+1,new Rotation2d(Math.PI*7/4)),
            new Pose2d(FieldConstants.blueRowOfAlgae+1,FieldConstants.yCenter,new Rotation2d(Math.PI*4/4)),
            new Pose2d(FieldConstants.blueRowOfAlgae,FieldConstants.yCenter-1,new Rotation2d(Math.PI*1/4)),
            new Pose2d(FieldConstants.blueRowOfAlgae+1,FieldConstants.yCenter+1,new Rotation2d(Math.PI*5/4)),
            new Pose2d(FieldConstants.blueRowOfAlgae+1,FieldConstants.yCenter-1,new Rotation2d(Math.PI*3/4)),
            new Pose2d(FieldConstants.blueRowOfAlgae+0.5,FieldConstants.yCenter,new Rotation2d(Math.PI*1/4)),
            new Pose2d(FieldConstants.blueRowOfAlgae,FieldConstants.yCenter,new Rotation2d(0.0))
        };

        Trajectory trajectory = createTrajectory(driveTrain, danceMoves, config);
        if (trajectory != null) {
            command = new PreussSwerveControllerCommand(
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
            new InstantCommand(() -> RobotContainer.setGyroAngleToStartMatch()),
            new InstantCommand(() -> RobotContainer.m_robotDrive.setDrivingMode(DrivingMode.PRECISION)),
            //new VerifyStartingPositionCommand(poseEstimatorSubsystem, danceMoves[0]),
            new InstantCommand(() -> poseEstimatorSubsystem.setCurrentPose(danceMoves[0])),
            new InstantCommand(() -> poseEstimatorSubsystem.field2d.setRobotPose(danceMoves[0])), // for debug
            new InstantCommand(() -> poseEstimatorSubsystem.field2d.getObject("trajectory").setTrajectory(trajectory)), // for debug
            command,
            new InstantCommand(() -> RobotContainer.m_PoseEstimatorSubsystem.field2d.setRobotPose(danceMoves[danceMoves.length-1])) // for debug
            // may need a driveToPose to perfectly position the robot.
            // will need some arm motion to socre the coral.
            // could add additional actions to grab an algea and drive to the processor and score there.
        );
    }
}
