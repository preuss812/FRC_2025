/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.CAN;
//import frc.robot.Constants.CANConstants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj.AnalogInput; // Assumes encoder connected to the Roborio.
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.utils.PreussMotorConfig;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANConstants {
        public static final int kSwerveRightRearRotate = 23;
        public static final int kSwerveRightRearDrive = 24;
        public static final int kSwerveRightRearCANCoder = 34;

        public static final int kSwerveRightFrontRotate = 21;
        public static final int kSwerveRightFrontDrive = 22;
        public static final int kSwerveRightFrontCANCoder = 32;

        public static final int kSwerveLeftRearRotate = 25;
        public static final int kSwerveLeftRearDrive = 26;
        public static final int kSwerveLeftRearCANCoder = 36;

        public static final int kSwerveLeftFrontRotate = 27;
        public static final int kSwerveLeftFrontDrive = 28;
        public static final int kSwerveLeftFrontCANCoder = 38;

        public static final int kElbowMotor1 = 43;
        public static final int kElbowMotor2 = 44;
        public static final int kAlgaeIntakeMotor = 42;
        public static final int kShoulderMotor = 40;

        public static final int kPDP = 0; // was 50 until 3/12/2024
        public static final int kPCM = 51;
    }

    
    public static final class OIConstants {
        public static final int kLeftJoystick = 0;
        public static final int kRightJoystick = 1;
        public static final int kControlBox = 2;
        

        // Controlbox interfaces
        public static final int kControlBoxPotX = 0;
        public static final int kControlBoxPotY = 1;
        public static final int[] kControlBox3WaySwitch = {1,2}; // 3 position switch, see truth table below
        public static final int kControlBoxTottleButton = 3; // normally closed (1), pressed is open (0)
        public static final int kControlBoxSw1 = 4; // up is (0), down is (1) for all two position switches
        public static final int kControlBoxSw2 = 5;
        public static final int kControlBoxSw3 = 6;
        public static final int kControlBoxSw4 = 7;

        /* 
        kControlBox3WaySwitch implements the following states
           sw1   |   sw2   | 3-way position
        ---------+---------+---------------------
            0    |    0    | Left
            0    |    1    | Center
            1    |    0    | No such position
            1    |    1    | Right
        */

        // Xbox joystick constants
        public static final int kXboxAButton = 1;
        public static final int kXboxBButton = 2;
        public static final int kXboxXButton = 3;
        public static final int kXboxYButton = 4;
        public static final int kXboxLBumper = 5;
        public static final int kXboxRBumper = 6;
        public static final int kXboxSelect = 7;
        public static final int kXboxStart = 8;
        public static final int kXboxLTrigger = 9;
        public static final int kXboxRTrigger = 10;

        public static final int kDriverControllerPort = 3;
        public static final double kDriveDeadband = 0.10;
    }
    public static final class AnalogIOConstants {
        public static final int kPressureTransducer = 0;
        public static final int kPressureOffset = -20;
        public static final int kPressureRange = 200;
    }

    public static final class PidConstants {
        public static final double kProportionalDriveStraight = 0.05;
     
        public static final double kElbow_kP = 2.7;
        public static final double kElbow_kI = 0.005;
        public static final double kElbow_IntegralZone=15;
        public static final double kElbow_kD = 0.0;
        public static final double kElbow_kF = 0.0;
        public static final double kElbow_rampRate = 0.5;
        public static final double kShoulder_kP = 2.7;
        public static final double kShoulder_kI = 0.005;
        public static final double kShoulder_IntegralZone=15;
        public static final double kShoulder_kD = 0.0;
        public static final double kShoulder_kF = 0.0;
        public static final double kArmExtension_kP = 0.3; //3.0;
        public static final double kArmExtension_kI = 0.0;
        public static final double kArmExtension_kD = 0.0;
        public static final double kArmExtension_kF = 0.0;
        public static final double kArmExtension_rampRate = 0.5;
        public static final double kPorportionalBalanceForward = 0.05;
        public static final double kProportionalBalanceBackward = 0.05;
        public static final double kAlgaeIntake_kP = 2.7; // TODO are these needed and tuned?
        public static final double kAlgaeIntake_kI = 0.0;
        public static final double kAlgaeIntake_kD = 0.0;
        public static final double kAlgaeIntake_kF = 0.0;
        
        public static final double kShooter_kP = 2.7; // TODO are these needed and tuned?
        public static final double kShooter_kI = 0.0;
        public static final double kShooter_kD = 0.0;
        public static final double kShooter_kF = 0.0; 
    }

    public static final class ElbowConstants {
   
        public static final double kElbowThreshold =2; // TODO - tune this value.  2 is too small but 10 may be too large - dph 2024-02-25.
        public static final double kElbowEncoderCountPerRevolution = 8192; 
        public static final double kElbowDegreesPerTick = 360.0/ElbowConstants.kElbowEncoderCountPerRevolution;
        public static final double kElbowTicksPerDegree = ElbowConstants.kElbowEncoderCountPerRevolution/360.0;
        
        // Positions are in Encode Ticks.
        // Ticks increase as the arm move down to the intake position.
        // The '0' position is defined by the scoring position which is enforced by the upper/reverse.
        // The 'max' position is defined by the arm fully rotated which is  enforced the forward limit switch.
        // The proper starting position for the arm is in the fully down position with the forward limit switch activated.
        public static final double kElbowMinPosition = 0;    // Smallest encoder value the software will rotate to.
        public static final double kElbowMaxPosition = 2800; // Largest encoder value the software will rotote to.
        public static final double kElbowRange = kElbowMaxPosition - kElbowMinPosition; // The number of ticks in the active range of arm motion between limits.

        public static final double kElbowStartingPosition = kElbowMaxPosition;  // We should start at the max position with the arm rotated down to intake Algaes.
        public static final double kElbowIntakePosition = kElbowMaxPosition;
        public static final double kElbowHookChainPosition = kElbowMaxPosition;
        public static final double kElbowScoringPosition = kElbowMinPosition;  // Rotated upward to score a Algae.

        public static final double kElbowPeakOutputForward =  0.80; // Limit output voltage to +/- 80% of the available voltage range.
        public static final double kElbowPeakOutputReverse = -0.80; // Limit output voltage to +/- 80% of the available voltage range.
        public static final double kElbowSensorUnitsPer100ms = kElbowRange*10.0;      // Untested: Max speed in MotionMagic mode.  Full range in 1 second.
        public static final double kElbowSensorUnitsPer100msPerSec = kElbowRange*10;  // Untested: Max acceleration in MotionMagicMode.  Full acceleration in 1 second

        public static final double kElbowRaiseTimeout = 2.0; // Seconds - for autonomous.
        public static final double kElbowLowerTimeout = 2.0; // Seconds - for autonomous.
        public static final double kElbowHomeSpeed = 0.2;    // Percent
        public static final double kElbowHomePosition = 0;    // ticks. TODO: Tune this value.
        public static final double kElbowLowAlgaePosition = 1000; // ticks. TODO: Tune this value.
        public static final double kElbowHighAlgaePosition = 2000; // ticks. TODO: Tune this value.
    }
    
    // Define locations on the field that may be useful for semi-automatic driving.
    public static final class FieldConstants {
        // All units in Meters.
        // These values are derived from april tag locations and field drawings in the game manual or cad drawings.
        public static final double robotInitialOrientation = 180.0;  // Assume robot starts on the starting line facing the reef.
        public static final double fieldLength = Units.inchesToMeters(657.37+33.51); // This is X
        public static final double fieldWidth = Units.inchesToMeters(317.15-0.15); // This is Y

        // Handy values
        public static double xMin = 0.00;
        public static double xMax = fieldLength; // Meters 
        public static double yMin = 0.00;
        public static double yMax = fieldWidth; // Meters
        public static double xCenter = (xMax - xMin)/2.0;
        public static double yCenter = (yMax - yMin)/2.0;
        public static double xSquareSize = xMax/8;
        public static double ySquareSize = yMax/4;
        public static double bargeWidth = Units.inchesToMeters(46.0);
        public static double tapeWidth = Units.inchesToMeters(2.0);
        public static double reefToStart = Units.inchesToMeters(88.0);
        public static double tapeToReef = Units.inchesToMeters(14.0);
        public static double zeroToReef = Units.inchesToMeters(144.0);
        public static double reefWidth = Units.inchesToMeters(93.50) - tapeToReef*2;
        public static double placedBallSpacing = Units.inchesToMeters(72.0);
        public static double bargeYCenter = Units.inchesToMeters(75.39);
        public static double checkFieldLength = zeroToReef*2 + reefToStart*2 + bargeWidth + tapeWidth*2 + 2*reefWidth;
        public static double startToBarge = (FieldConstants.xMax - checkFieldLength)/2.0;
        
        // Handy X coordinates:
        public static double blueRowOfAlgae = Units.inchesToMeters(48.0);
        public static double blueStartLine =  zeroToReef + reefWidth + reefToStart + tapeWidth/2;
        public static double redRowOfAlgae = xMax - blueRowOfAlgae;
        public static double redStartLine = xMax - blueStartLine;

        // Handy Y coordinates:
        public static double stagedCoralAlgaeLow = yCenter - Units.inchesToMeters(72);
        public static double stagedCoralAlgaeMed = yCenter;
        public static double stagedCoralAlgaeHigh = yCenter + Units.inchesToMeters(72);

        // Helper functions to translate Blue coordinates to Red coordinates.
        // This is most likely to be used for creating red alliance autonomous routines
        // from blue alliance autonomous routines.  For 2025 this is a rotation about the center
        // of the field of play.  In 2024 this was a mirroring of the field about the center of the field.

        /**
         * Transform a Translation2d for the blue alliance to the complementary Translation2d for the red alliance.
         * @param blueRotation - the Translation2d for the robot in the blue alliance.
         * @return - the Translation2d for the robot in the red alliance.
         */public static Translation2d BlueToRedTranslation(Translation2d blueTranslation) {
            return new Translation2d(xMax - blueTranslation.getX(), yMax - blueTranslation.getY());
        }

        /**
         * Transform a Rotation2d for the blue alliance to the complementary Rotation2d for the red alliance.
         * @param blueRotation - the Rotation2d for the robot in the blue alliance.
         * @return - the Rotation2d for the robot in the red alliance.
         */
        public static Rotation2d BlueToRedRotation(Rotation2d blueRotation) {
            return blueRotation.rotateBy(new Rotation2d(Math.PI));
        }

        /**
         * Transform a Pose2d for the blue alliance to the complementary Pose2d for the red alliance.
         * @param bluePose - The Pose2d for the robot in the blue alliance.
         * @return         - The Pose2d for the robot in the red alliance.
         */
        public static Pose2d BlueToRedPose(Pose2d bluePose) {
            return new Pose2d(BlueToRedTranslation(bluePose.getTranslation()), BlueToRedRotation(bluePose.getRotation()));
        }

        // Field Coordinate transformations for alliances.
        public static int BlueAlliance = 0;
        public static int RedAlliance = 1;
        public static Pose2d blueToRedTransform = new Pose2d(xMax, yMax, new Rotation2d(Math.PI));
    }
    
    public static final class AlgaeIntakeConstants {
        public static final double kPickUpAlgaeSpeed = 1.0; // TODO needs tuning.
        public static final double kExpelAlgaeSpeed = -1.0; // TODO needs tuning.
        public static final int kLimitSwitchChannel = 2;
    }

    public static final class BrakeConstants {
        public static final String kNotBraking = "NotBraking";
        public static final String kUnknown    = "Unknown";
        public static final String kBraking    = "Braking";
    }

    public static final class VisionConstants {
        /**
        * Physical location of the camera on the robot, relative to the center of the robot.
        */
        // Values in Meters.
        public static final Transform3d XCAMERA_TO_ROBOT =
        // x pos or neg doesn't get us where we want to go      w/apriltag 1    
        //new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d()); // Before we rotated the RoboRio
        // new Transform3d(new Translation3d(0.0, -0.3425, -0.233), new Rotation3d());
        //new Transform3d(new Translation3d(Units.inchesToMeters(-10.5), Units.inchesToMeters(0.0), -0.233), new Rotation3d());
        // worked nicely 2024-01-16 1700     w/ap-riltag 1       new Transform3d(new Translation3d(0, 0.3425, -0.233), new Rotation3d());

        // Camera is located 10.5 inches behind the center of the robot,
        // 0 meters offset to the side, 
        // 0.233 meters above the ground.
        // TODO Check values and signs of values.
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.5),
                Units.inchesToMeters(0.0),
                Units.inchesToMeters(0.0) // Distance is 10.5 inches but emperically 0 works better.
            ),
            new Rotation3d(0.0,0.0, Math.PI)
        );
        public static double cameraXOffsetToRobot = Units.inchesToMeters(-12.5);
        public static double cameraYOffsetToRobot = Units.inchesToMeters(0.0);
        public static double cameraHeightToGround = Units.inchesToMeters(0.0);
        public static double cameraRoll  = Units.degreesToRadians(0.0);
        public static double cameraPitch = Units.degreesToRadians(0.0); 
        public static double cameraYaw   = Units.degreesToRadians(180.0); // Rear facing camera.
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
            new Translation3d(cameraXOffsetToRobot,cameraYOffsetToRobot,cameraHeightToGround),
            new Rotation3d(cameraRoll, cameraPitch, cameraYaw)
        );
        public static Pose2d nearBlueProcessor = new Pose2d(Units.inchesToMeters(235.73), Units.inchesToMeters(1.5), new Rotation2d(Units.degreesToRadians(270)));
        public static Pose2d nearRedProcessor = new Pose2d(Units.inchesToMeters(455.15), Units.inchesToMeters(217.15-1.5), new Rotation2d(Units.degreesToRadians(90)));
        
        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
        public enum AprilTag {
            UNKNOWN(0),
            RED_PROCESSOR(3),
            BLUE_PROCESSOR(16);
            
            
            private int id;
            private AprilTag(int id) {
                this.id = id;
            }
            public int id() {return id; }

        }
    
    }

    public static final class ShoulderConstants {
        public static final AnalogInput kShoulderEncoderInputChannel = new AnalogInput(3);
        public static final int kPidIdx = 0;
        public static final int kTimeoutMs = 10;
        public static final double kShoulderStartingPosition = 0;  // We should start at the minimum position with the arm rotated down.
        public static final double kShoulderPeakOutputForward =  0.5; // TODO: Tune this value.
        public static final double kShoulderPeakOutputReverse = -0.5; // TODO: Tune this value.
        public static final double kShoulderTimeout = 3.0; // Seconds to wait for the arm to rotate to the shooting position.
        public static final double kShoulderMinPosition = 0;    // Smallest encoder value the software will rotate to. TODO: Tune this value.
        public static final double kShoulderMaxPosition  = 2000; // Largest encoder value the software will rotote to. TODO: Tune this value.
        public static final double kShoulderRotationThreshold = 20; // ticks. TODO: Tune this value.
        public static final double kShoulderProcessorPosition = 200; // ticks. TODO: Tune this value.
        public static final double kShoulderHomeSpeed = 0.2; // Percent. TODO: Tune this value.
        public static final double kShoulderHomePosition = 0; // ticks. TODO: Tune this value.
        public static final double kShoulderLowAlgaePosition = 1000; // ticks. TODO: Tune this value.
        public static final double kShoulderHighAlgaePosition = 2000; // ticks. TODO: Tune this value.
    }

    public static final class RotationConstants {
        public static final Rotation2d zero = new Rotation2d(0);
        public static final Rotation2d rotate180 = new Rotation2d(Math.PI);
    }


    public static final class CameraConstants {
        public static final String kCamName="pv-812";
    }

    public static final int kBrakeLightRelay = 0;

    // The constants in DriveConstants and ModuleConstants are from 
    // https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java
    // Which is where I sourced the MAXSwerve template - dph - 2024-01-12
    public static final class DriveConstants {
        // Driving Parameters - Algae that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMaxSpeedMetersPerSecond = 4.5; //4.5; 1.0 in the lab // Limit how violently swerve works
        public static final double kMaxAngularSpeed = 4 * Math.PI; // radians per second
    
        public static final double kDirectionSlewRate = 1.2; // radians per second
        public static final double kMagnitudeIncreaseSlewRate = 0.8; // percent per second (1 = 100%)
        public static final double kMagnitudeDecreaseSlewRate = 3.6; // percent per second (1 = 100%)
        public static final double kRotationalIncreaseSlewRate = 0.4;// 2.0; // percent per second (1 = 100%) // UNDO
        public static final double kRotationalDecreaseSlewRate = 4.0;

        // Driving Parameters for PRECISION DrivingMode.  This is a slower mode for more precise positioning.
        public static final double kMaxSpeedMetersPerSecondPM = 1.6; //4.5; 1.0 in the lab // Limit how violently swerve works
        public static final double kMaxAngularSpeedPM = 1 * Math.PI; // radians per second
    
        public static final double kDirectionSlewRatePM = 0.6; // radians per second
        public static final double kMagnitudeIncreaseSlewRatePM = 0.45; // percent per second (1 = 100%)
        public static final double kMagnitudeDecreaseSlewRatePM = 3.6; // percent per second (1 = 100%)
        public static final double kRotationalIncreaseSlewRatePM = 0.3;// 2.0; // percent per second (1 = 100%) // UNDO
        public static final double kRotationalDecreaseSlewRatePM = 4.0;
        
        // Chassis configuration
        public static final double kTrackWidth = Units.inchesToMeters(17.75); // was 26.5 until 3/5/2024, actual is 17.75
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(17.75); //  was 26.5 until 3/5/2024, actual is 17.75
        // Distance between front and back wheels on robot

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2 + Units.inchesToMeters(6.0), kTrackWidth / 2),
            new Translation2d(kWheelBase / 2 + Units.inchesToMeters(6.0), -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset =0;// -Math.PI / 4;
        public static final double kFrontRightChassisAngularOffset = 0;//Math.PI / 4;
        public static final double kBackLeftChassisAngularOffset = 0;//5 * Math.PI / 4;
        public static final double kBackRightChassisAngularOffset =0;// 3 * Math.PI / 4;
    
        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = CANConstants.kSwerveLeftFrontDrive;
        public static final int kRearLeftDrivingCanId = CANConstants.kSwerveLeftRearDrive;
        public static final int kFrontRightDrivingCanId = CANConstants.kSwerveRightFrontDrive;
        public static final int kRearRightDrivingCanId = CANConstants.kSwerveRightRearDrive;
    
        public static final int kFrontLeftTurningCanId = CANConstants.kSwerveLeftFrontRotate;
        public static final int kRearLeftTurningCanId = CANConstants.kSwerveLeftRearRotate;
        public static final int kFrontRightTurningCanId = CANConstants.kSwerveRightFrontRotate;
        public static final int kRearRightTurningCanId = CANConstants.kSwerveRightRearRotate;

        public static final int kFrontLeftTurningEncoderCanId = CANConstants.kSwerveLeftFrontCANCoder;
        public static final int kRearLeftTurningEncoderCanId = CANConstants.kSwerveLeftRearCANCoder;
        public static final int kFrontRightTurningEncoderCanId = CANConstants.kSwerveRightFrontCANCoder;
        public static final int kRearRightTurningEncoderCanId = CANConstants.kSwerveRightRearCANCoder;

        public static final boolean kGyroReversed = false;

        public static final double kBackToCenterDistance = Units.inchesToMeters(17.5); //was 15.0 until 3/5/2024
        public static final double kBumperWidth = Units.inchesToMeters(3.0);
        public static final double kRobotWidth = Units.inchesToMeters(24.0+kBumperWidth*2.0); // Frame width plus 2 bumpers.
        public static final double kRobotLength = Units.inchesToMeters(24.0+6); // Frame length plus 2 bumpers.
        public static final double kApproximateStartingY = FieldConstants.yMax - Units.inchesToMeters(36.0); // Meters (ie near the amp)
        public static final double kStartingOrientation = 0.0; // Starting orientation in radians (ie robot back against the alliance wall)
        public static final Translation2d robotCenterToFrontBumper = new Translation2d(kRobotLength/2 + kBumperWidth, 0);
        public static final Translation2d robotCenterToBackBumper = new Translation2d(-(kRobotLength/2 + kBumperWidth), 0);
        public static final Rotation2d rotate180 = new Rotation2d(Math.PI);
        public static final Translation2d rotatedRobotFrontBumper(Rotation2d rotation) {
            return robotCenterToFrontBumper.rotateBy(rotation);
        }
        // Return a pose for the robot to be facing the pose with the center of it's front bumper touching the pose
        public static Pose2d robotFrontAtPose(Pose2d pose) {
            Translation2d position = pose.getTranslation().plus(rotatedRobotFrontBumper(pose.getRotation()));
            Rotation2d rotation = pose.getRotation().rotateBy(rotate180);
            return new Pose2d(position, rotation);
        }
      }
    
      public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;
    
        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;
    
        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0); // wheels 4" x 1.5"
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = 8.14; //(45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
    
        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second
    
        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians
        public static final double kTurningSRXEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // rotations

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;
    
        public static final double kTurningP = 0.3;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;
    
        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;
    
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
      }

      public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
        public static final double kRotationSpeed = -0.05;
    
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static String[] mode = {"PV-Score-Leave", "Leave", "US-Score-Leave","Do Nothing"};
        public static final int PVScoreLeaveMode = 0;
        public static final int LeaveMode = 1;
        public static final int USScoreLeaveMode = 2;
        public static final int DoNothingMode = 3;
        public static final int DefaultMode = 0;
        
      }
    
      public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
      }

    public static final class ColorConstants {
        /* Name: Construction Cone Orange
            URL:  https://www.computerhope.com/cgi-bin/htmlcolor.pl?c=F87431
            RGB:  248, 116, 49
            HSL:  20.20-deg, 93.43%, 58.24%
            HEX: #F87431

            wpilib/util/Color
            https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/util/Color.html
            There are two constructor signatures available, one for
            Integer values and nother for Double values. If you send a
            Double, the values must be adjusted to a 0 to 1 range from
            0 to 255 when passed as integers. Doubles are used so that we 
            can better control precision if needed.
        */
        public static final double[] kAlgaeTargetRBG = { 248/255.0, 49/255.0, 116/255.0 }; // TODO Verify color code // 2020 code used RBG not RGB????

        public static final double kColorConfidenceThreshold = 0.90;
        public static final int     kColorProximityThreshold = 300; // higher closer, lower is further away
    }

    public static final class UltrasonicConstants {
        public static final int kUltrasonicAnalogPort = 3;
        public static int kPingChannel = 0;
        public static int kEchoChannel = 1;
        public static double kOffsetToBumper = 0.157; // Meters
    }

    public static PreussMotorConfig algaeMotorConfig = new PreussMotorConfig(CANConstants.kAlgaeIntakeMotor)
        .setP(PidConstants.kAlgaeIntake_kP)
        .setI(PidConstants.kAlgaeIntake_kI)
        .setD(PidConstants.kAlgaeIntake_kD)
        .setF(PidConstants.kAlgaeIntake_kF);

    public static final PreussMotorConfig shoulderMotor = new PreussMotorConfig(CANConstants.kShoulderMotor)
    .setP(PidConstants.kShoulder_kP)
        .setP(PidConstants.kShoulder_kI)
        .setP(PidConstants.kShoulder_kD)
        .setP(PidConstants.kShoulder_kF)
        .setP(PidConstants.kShoulder_IntegralZone);
        
    public static final PreussMotorConfig elbowMotor1 = new PreussMotorConfig(CANConstants.kElbowMotor1)
        .setP(PidConstants.kElbow_kP)
        .setP(PidConstants.kElbow_kI)
        .setP(PidConstants.kElbow_kD)
        .setP(PidConstants.kElbow_kF)
        .setP(PidConstants.kElbow_IntegralZone);
    
    public static final PreussMotorConfig elbowMotor2 = new PreussMotorConfig(CANConstants.kElbowMotor2)
        .setP(PidConstants.kElbow_kP)
        .setP(PidConstants.kElbow_kI)
        .setP(PidConstants.kElbow_kD)
        .setP(PidConstants.kElbow_kF)
        .setP(PidConstants.kElbow_IntegralZone);


}
