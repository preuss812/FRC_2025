// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraVisionSubsystem;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.utils.Line;
import frc.utils.DrivingConfig;
import frc.utils.PreussAutoDrive;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;

public class DriveOnAprilTagProjectionCommand extends Command {
  
  /** 
   * Creates a new command to move the robot along the line that projects perpendicularly from an april tag.
   */
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DriveSubsystemSRX robotDrive;
  private final PhotonCamera visionSubsystem;
  private final XboxController xbox;
  private final DrivingConfig config;
  private final PreussAutoDrive autoDrive;
  private final boolean simulation;
  private Pose2d aprilTagPose = null;
  private double cameraToRobotAngle;
  private boolean commandIsActive = false;
  
  private double p; // The constant P from the normal form for the april tag projection line.
  private double xSign; // The sign of x axis motion toward the april tag.
  private double ySign; // The sign of y axis motion toward the april tag. 
  private double projectionM;
  private double projectionB;
  private static double sqrtOfTwoOverTwo = Math.sqrt(2.0)/2.0;
  private boolean debug = true;
  private static final double[] simulatedJoystick = new double[] {0.8,0.1,0.1};
  private int simulationCycleCount = 0;
  private int simulationMinCount = -500;
  private int simulationMaxCount = 500;
  private static final Pose2d simulationStartingPose = new Pose2d(15.5, 2.6, new Rotation2d(Math.PI));
  private static final double needToBeOnLineDistance = 1.0; // (meters)
  private Translation2d lastThrottle = new Translation2d(0,0);

  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params photonCamera - the Photon Camera Subsystem used to see the April Tags.
   */
  public DriveOnAprilTagProjectionCommand(
      PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , XboxController xbox
    , DrivingConfig config
    , boolean simulation) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.visionSubsystem = photonCamera;
    this.xbox = xbox;
    this.config = config;
    this.simulation = simulation;
    autoDrive = new PreussAutoDrive(robotDrive, poseEstimatorSubsystem, config, simulation);
    cameraToRobotAngle = VisionConstants.XCAMERA_TO_ROBOT.getRotation().getZ();

    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Apply an exponential smoothing
  private Translation2d smoother(Translation2d throttle) {
    double smoothingFactor = 0.2; // 20% new value, 80% old
    Translation2d result =  new Translation2d(
      throttle.getX() * smoothingFactor  + lastThrottle.getX() * (1.0-smoothingFactor),
      throttle.getY() * smoothingFactor  + lastThrottle.getY() * (1.0-smoothingFactor) );
    this.lastThrottle = new Translation2d(throttle.getX(), throttle.getY());
    return result;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("DP","Active");
    int fiducialId = CameraVisionSubsystem.NO_TAG_FOUND;
    autoDrive.reset();
    aprilTagPose = null;
    commandIsActive = false;
    p = 0.0;
    xSign = 0.0;
    ySign = 0.0;
    
    if (simulation) {
      fiducialId = 6;
      simulationCycleCount = simulationMinCount;
      autoDrive.setCurrentPose(simulationStartingPose);
    } else {
      fiducialId = poseEstimatorSubsystem.getBestAprilTag(0.2); 
    }

    if (fiducialId != CameraVisionSubsystem.NO_TAG_FOUND) {
          // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
      commandIsActive = true;
      
      // Compute the projected line from the april tag.
      // This y = mx+b will be used in execute() to save compute cycles
      // I'm using the "normal form" of the line.
      double theta = aprilTagPose.getRotation().getRadians();
      
      SmartDashboard.putNumber("DP theta", theta);
      SmartDashboard.putNumber("DP cos", Math.cos(theta));

      p = aprilTagPose.getX()* Math.cos(theta) + aprilTagPose.getY() * Math.sin(theta);
      SmartDashboard.putNumber("DP p", p);

      // We also need to determine which direction we are moving toward the line.
      // In other words, which way is towards the target along the line.
      double cos = Math.cos(theta);
      double sin = Math.sin(theta);
      final double epsilon = 0.0001;
      if (Math.abs(cos) < epsilon) cos = 0.0;
      if (Math.abs(sin) < epsilon) sin = 0.0;
      xSign = -Math.signum(cos);
      ySign = -Math.signum(sin);
      if (xSign !=0 ) {
        projectionM = Math.tan(theta);
        // y = mx+b => y - mx = b;
        projectionB = aprilTagPose.getY() - projectionM * aprilTagPose.getX();
      }
      SmartDashboard.putNumber("DP x", xSign);
      SmartDashboard.putNumber("DP y", ySign);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!commandIsActive) return; // If we did not find an april tag, prevent problems with uninitialized variables below

    Translation2d fieldThrottle;
    Pose2d robotPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    Translation2d aprilThrustVector;
    Translation2d fieldThrustVector;
    double throttle;
    double desiredRotation;
    double rotationError;
    

    if (aprilTagPose == null)  return;

    robotPose = autoDrive.getCurrentPose();

    throttle = -xbox.getRightY(); // Driver controls +/-X on the projection line
    if (simulation) {
      simulationCycleCount++;
      if (robotPose.getX() < FieldConstants.xMin 
       || robotPose.getX() > FieldConstants.xMax 
       || robotPose.getY() < FieldConstants.yMin 
       || robotPose.getY() > FieldConstants.yMax
      ) {
        if (simulationCycleCount > 0) {
          simulationMaxCount = simulationCycleCount - 1;
          simulationMinCount = -simulationMaxCount;
        }
        else {
          simulationMinCount = simulationCycleCount + 1;
          simulationMaxCount = -simulationMinCount;
        }
      }
      if (simulationCycleCount > simulationMaxCount) {
        simulationCycleCount = simulationMinCount;
      }
      if (simulationCycleCount < 0) {
        throttle = -simulatedJoystick[0];
      } else {
        throttle = simulatedJoystick[0];
      }
    }
    //rotationSpeed = MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband); // Based on xbox right joystick.
    // ? Should rotation be an automatic calculate to keep the robot facing the tag?

    SmartDashboard.putNumber("DA throttle", throttle);
    
    // Calculate rotation required for the camera to face the april tag.
    desiredRotation = MathUtil.angleModulus(Utilities.getHeading(robotPose.getTranslation(), aprilTagPose.getTranslation() )+ cameraToRobotAngle);
    rotationError = MathUtil.angleModulus(desiredRotation - robotPose.getRotation().getRadians());
    rotationSpeed = autoDrive.calculateClampedRotation(rotationError);

    Translation2d robotInAprilTagSpace = new Translation2d(robotPose.getX() - aprilTagPose.getX(), robotPose.getY() - aprilTagPose.getY()).rotateBy(aprilTagPose.getRotation().times(-1.0));

    double epsilon = 0.05; // 5cm
    epsilon = 1.0; // debug, a test
    if (throttle > 0) {
      // We are moving away from the target.
      // Therefore the target position will be diagonally toward the projection lihe, Y = 0 in april tag space.
      
      if (robotInAprilTagSpace.getY() > epsilon) {
        // we need to move down toward the projection
        aprilThrustVector = new Translation2d(-sqrtOfTwoOverTwo, sqrtOfTwoOverTwo);
        SmartDashboard.putString("DOA Path", "+ +");
      } else if (robotInAprilTagSpace.getY() > -epsilon) {
        // On the line, stay on the line.
        if (robotInAprilTagSpace.getX() < 0.0) {
          // We are "behind" the tag.  move forward along the projection line.
          aprilThrustVector = new Translation2d(-1.0, 0);
          SmartDashboard.putString("DOA Path", "+ 0 -");
        } else {
          double heading = Math.atan2(robotInAprilTagSpace.getY(), robotInAprilTagSpace.getX());
          //double r = Line.vectorLength(robotInAprilTagSpace);
          aprilThrustVector = new Translation2d(-Math.cos(heading), Math.sin(heading)); 
          SmartDashboard.putString("DOA Path", "+ 0 "+Units.radiansToDegrees(heading));
        }
      } else {
        // we need to move up toward the projection
        aprilThrustVector = new Translation2d(-sqrtOfTwoOverTwo, -sqrtOfTwoOverTwo);
        SmartDashboard.putString("DOA Path", "+ -");
      }
    } else {
      // We are moving toward from the target.
      // Therefore the target position will be diagonally toward the projection lihe, Y = 0 in april tag space.
      if (Math.abs(robotInAprilTagSpace.getX()) > Math.abs(robotInAprilTagSpace.getY())) {
        // We are far enough away we can move diagonally
        if (robotInAprilTagSpace.getY() > epsilon) {
          // we need to move down toward the projection
          SmartDashboard.putString("DOA Path", "- + +");
          aprilThrustVector = new Translation2d(-sqrtOfTwoOverTwo, -sqrtOfTwoOverTwo);
        } else if (robotInAprilTagSpace.getY() > -epsilon) {
          // On the line, stay on the line.
          double heading = Math.atan2(-robotInAprilTagSpace.getY(), robotInAprilTagSpace.getX());
          //ouble r = Line.vectorLength(robotInAprilTagSpace);
          aprilThrustVector = new Translation2d(-Math.cos(heading), Math.sin(heading)); 
          SmartDashboard.putString("DOA Path", "- + 0");
        } else {
          // we need to move up toward the projection
          aprilThrustVector = new Translation2d(-sqrtOfTwoOverTwo, sqrtOfTwoOverTwo);
          SmartDashboard.putString("DOA Path", "- + -");
          simulationMinCount = simulationCycleCount + 1;
        }
      } else {
        //  We are wider away from the target so we need to exert more centering force.
        // We are far enough away we can NOT move diagonally
        // Ideally, this would rarely come up.
        if (robotInAprilTagSpace.getY() > epsilon) {
          // head straight down to the projection line
          aprilThrustVector = new Translation2d(0.0, -1);
          SmartDashboard.putString("DOA Path", "- - +");
        } else if (robotInAprilTagSpace.getY() > -epsilon) {
          // On the line, stay on the line.
          double r = Line.vectorLength(robotInAprilTagSpace);
          aprilThrustVector = new Translation2d(-Math.cos(robotInAprilTagSpace.getX()), Math.sin(robotInAprilTagSpace.getY()/r)); 
          SmartDashboard.putString("DOA Path", "- - 0");
        } else {
          // we need to move up toward the projection
          aprilThrustVector = new Translation2d(0.0, 1.0);
          SmartDashboard.putString("DOA Path", "- - -");
        }
      }
    }
    // Rotate the thrust vector into field coordinates and use that as xSpeed, ySpeed
    //fieldThrustVector = aprilThrustVector.rotateBy(aprilTagPose.getRotation().times(-1.0).rotateBy(new Rotation2d(Math.PI)));
    fieldThrustVector = aprilThrustVector.rotateBy(aprilTagPose.getRotation().times(1.0).rotateBy(new Rotation2d(0.0)));
    fieldThrottle = fieldThrustVector.times(throttle);  // Minus puts back into yaw, pitch
    //fieldThrottle = smoother(fieldThrottle); // Maybe try on a robot.
    if (simulation) {
      if (throttle < 0 && Line.vectorLength(robotInAprilTagSpace) < 0.5) {
        fieldThrottle = fieldThrottle.times(0.5); // Just to reduce driving through the tags
      }
    }
    SmartDashboard.putNumber("DOA thetaF", Units.radiansToDegrees(Math.atan(fieldThrustVector.getY()/fieldThrustVector.getX())));
    SmartDashboard.putNumber("DOA thetA", Units.radiansToDegrees(Math.atan(robotInAprilTagSpace.getY()/robotInAprilTagSpace.getX())));

    if (debug) SmartDashboard.putNumber("DA X", fieldThrottle.getX());
    if (debug) SmartDashboard.putNumber("DA Y", fieldThrottle.getY());
    
    xSpeed = fieldThrottle.getX();
    ySpeed = fieldThrottle.getY();
    //xSpeed = autoDrive.calculateClampedX(fieldThrottle.getX());
    //ySpeed = autoDrive.calculateClampedY(fieldThrottle.getY());
  
    if (debug) SmartDashboard.putNumber("DA xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("DA ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("DA rSpeed", rotationSpeed);
    autoDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoDrive.drive(0, 0, 0, true, true);
        SmartDashboard.putString("DP","Done");
    commandIsActive = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (aprilTagPose == null);
  }

  public boolean isActive() {
    return commandIsActive;
  }
}
