// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.PhotonCamera;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities;

public class GotoAprilTagCommand extends Command {
  
  public class GotoAprilTagConfig {
    private double maxThrottle;
    private double linearP;
    private double linearI;
    private double linearD;
    private double linearF;
    private double linearIZone;
    private double linearTolerance;

    private double maxRotation;
    private double angularP;
    private double angularI;
    private double angularD;
    private double angularF;
    private double angularIZone;
    private double angularTolerance;

    /**
     * default constructor
     */
    public GotoAprilTagConfig() {
      maxThrottle = 0.80;
      linearP = 2.0;
      linearI = 0.0; // linearP/100.0;
      linearD = 0.0; // linearP*10.0;
      linearF = 0.0;
      linearIZone = Units.inchesToMeters(4.0);
      linearTolerance = Units.inchesToMeters(2.0);

      maxRotation = 0.50;
      angularP =  1.0; // 0.35;
      angularI = 0.0; // angularI/100.0;
      angularD = 0.0; //angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(2.0);
    }
    public GotoAprilTagConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public GotoAprilTagConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public GotoAprilTagConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public GotoAprilTagConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public GotoAprilTagConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public GotoAprilTagConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public GotoAprilTagConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public GotoAprilTagConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public GotoAprilTagConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public GotoAprilTagConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public GotoAprilTagConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public GotoAprilTagConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public GotoAprilTagConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public GotoAprilTagConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

    public double getMaxThrottle() { return maxThrottle; }
    public double getLinearP() { return linearP; }
    public double getLinearI() { return linearI; }
    public double getLinearD() { return linearD; }
    public double getLinearF() { return linearF; }
    public double getLinearIZone() { return linearIZone; }
    public double getLinearTolerance() { return linearTolerance; }
    
    public double getMaxRotation() { return maxRotation; }
    public double getAngularP() { return angularP; }
    public double getAngularI() { return angularI; }
    public double getAngularD() { return angularD; }
    public double getAngularF() { return angularF; }
    public double getAngularIZone() { return angularIZone; }
    public double getAngularTolerance() { return angularTolerance; }
  } // DebugGotoAprilTagConfig Class

  /** Creates a new command to move the robot to the specified pose. */
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DriveSubsystemSRX robotDrive;
  private final PhotonCamera photonCamera;
  private final GotoAprilTagConfig config;
  private final double targetDistance;

  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private Pose2d targetPose;
  private Pose2d aprilTagPose;
  private boolean onTarget;
  private double cameraToRobotAngle; // Radians the rotation to get from the camera to the front of the robot.
  private boolean debug = true; // turn on/off SmartDashBoard feedback
  private boolean simulatingRobot = true; // force robot starting position and april tag number for debugging purposes.
  private int simulationNumber = 0;
  private Pose2d simulatedRobotPose;
  private int[] simulationAprilTagIDs = new int [] {
    6,6,6,
    7,7,7,
    19,19,19,
    17,17,17
  };
  private Pose2d[] simulatedRobotStartingPoses = new Pose2d[] {
  new Pose2d(12.5, 0.60, new Rotation2d(Units.degreesToRadians(-60))),
  new Pose2d(15.5, 0.60, new Rotation2d(Units.degreesToRadians(-60))),
  new Pose2d(15.5, 2.60, new Rotation2d(Units.degreesToRadians(-60))),

  new Pose2d(14.5, 7.60, new Rotation2d(Units.degreesToRadians(0))),
  new Pose2d(15.5, 5.60, new Rotation2d(Units.degreesToRadians(10))),
  new Pose2d(15.5, 2.60, new Rotation2d(Units.degreesToRadians(-10))),
  
  new Pose2d(3.5, 7.60, new Rotation2d(Units.degreesToRadians(110))),
  new Pose2d(1.5, 7.60, new Rotation2d(Units.degreesToRadians(130))),
  new Pose2d(1.5, 5.60, new Rotation2d(Units.degreesToRadians(130))),

  new Pose2d(1.5, 2.60, new Rotation2d(Units.degreesToRadians(-125))),
  new Pose2d(1.5, 0.60, new Rotation2d(Units.degreesToRadians(-130))),
  new Pose2d(4.5, 0.60, new Rotation2d(Units.degreesToRadians(-120)))
  };



  private double maximumAmbiguity = 0.4; // Should be lower.
  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params photonCamera - the Photon Camera Subsystem used to see the April Tags.
   * @params targetDistance - the distance in meters to be away from the April Tag.
   */
  public GotoAprilTagCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , double targetDistance
    , boolean simulatingRobot) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.targetDistance = targetDistance;
    this.simulatingRobot = simulatingRobot;
    onTarget = false;
    this.config = new GotoAprilTagConfig();
    addRequirements(robotDrive, poseEstimatorSubsystem);
    cameraToRobotAngle = VisionConstants.XCAMERA_TO_ROBOT.getRotation().getZ();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = true; // Not really but if we dont find a target, this will cause the command to end immediately.
    //debugPID = RobotContainer.m_BlackBox.isSwitchCenter();
    double linearP = config.getLinearP();
    double linearI = config.getLinearI();
    
    var pipelineResult = photonCamera.getLatestResult();
    if (pipelineResult.hasTargets() || simulatingRobot) {
      int fiducialId = -1; // Sentinel value

      // If we are simulating the robot, use the predefined aprilTag and robot poses cycling through the poses with each successive command.
      if (simulatingRobot) {
        fiducialId = simulationAprilTagIDs[simulationNumber];
        simulatedRobotPose = simulatedRobotStartingPoses[simulationNumber++];
        poseEstimatorSubsystem.setCurrentPose(simulatedRobotPose);
        aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
        if  (simulationNumber >= simulatedRobotStartingPoses.length) {
          simulationNumber = 0;
        }
      } else {
        // Use the vision subsystem to get the april tag that we have the best view of
        // The assumption here is that the driver wants to go that tag.
        var target = pipelineResult.getBestTarget();
        if (debug) SmartDashboard.putNumber("TagAmbiguity", target.getPoseAmbiguity());
        

        // If we have a tag and can see it well, use it. Otherwise, the command will just end.
        if (target.getPoseAmbiguity() <= maximumAmbiguity)  { 
          SmartDashboard.putString("DebugG2A", "Pass");
          fiducialId = target.getFiducialId();
        } else {
          SmartDashboard.putString("DebugG2A", "Fail");
          if (debug) SmartDashboard.putNumber("TagAmbiguity", target.getPoseAmbiguity());


        }
      }
      if (debug) SmartDashboard.putNumber("TagNumber", fiducialId);
      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      if (fiducialId >= 0) {
        aprilTagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
        targetPose = Utilities.backToPose(aprilTagPose, targetDistance);

        xController = new PIDController(
          linearP, // config.getLinearP(),
          linearI, // config.getLinearI(),
          config.getLinearD()
        );
        xController.setIZone(config.getLinearIZone());
        yController = new PIDController(
          linearP, // config.getLinearP(),
          linearI, // config.getLinearI(),
          config.getLinearD()
        );
        yController.setIZone(config.getAngularIZone()); // TODO Needs Tuning.

        rotationController = new PIDController(
          config.getAngularP(),
          config.getAngularI(),
          config.getAngularD()
          );
        rotationController.setTolerance(config.getAngularTolerance()); // did not work, dont understand yet
        rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
        onTarget = false;
        if (debug) SmartDashboard.putNumber("G2A dist", targetDistance);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (onTarget) return; // Prevent code from running if we are there or if there was no apriltag.

    boolean controlRotation = true; // TODO test with true.
    Translation2d translationErrorToTarget;
    double rotationError;
    Pose2d robotPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    // get the robot's position on the field.
    robotPose = poseEstimatorSubsystem.getCurrentPose();
    
    // Force the robot position and rotation to known values for debugging purposes.
    if (simulatingRobot) {
        robotPose = simulatedRobotPose;
        poseEstimatorSubsystem.setCurrentPose(robotPose);
    }

    // Calculate the X and Y offsets to the target location
    translationErrorToTarget = new Translation2d( targetPose.getX() - robotPose.getX(), targetPose.getY() - robotPose.getY());

    // Control rotation to always face directly at the target as it approaches.
    // This is to ensure that we get the most updates to our pose estimator for best possitioning accuracy.
    // Calculate the tangent of the translation error.
    double desiredRotation;
    double errorVectorMagnitude = Math.pow(Math.pow(translationErrorToTarget.getX(),2) + Math.pow(translationErrorToTarget.getY(),2),0.5);
    desiredRotation = MathUtil.angleModulus(Utilities.getHeading(robotPose.getTranslation(), aprilTagPose.getTranslation() )+ cameraToRobotAngle);
    // If we are very close to the target, use the target pose.
    if (errorVectorMagnitude  < 0.5 /* meters */) {
      desiredRotation = targetPose.getRotation().getRadians();
    }
    rotationError = MathUtil.angleModulus(desiredRotation - robotPose.getRotation().getRadians());

    // Make sure the rotation error is between -PI and PI
    if (debug) SmartDashboard.putNumber("G2A R", Units.radiansToDegrees(rotationError));
    if (debug) SmartDashboard.putNumber("G2A X", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("G2A Y", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < config.getLinearTolerance()
    &&  ((!controlRotation) || (Math.abs(rotationError) < config.getAngularTolerance()))) {
      // We are close enough.  Stop the robot and the command.
      if (debug) SmartDashboard.putBoolean("G2A OnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // We are not close enough yet./

      // Transform the error into apriltag coordinates
      Translation2d aprilTagErrorVector = translationErrorToTarget.rotateBy(targetPose.getRotation()); //  vector from apriltag to the robot rotated in april tag space

      // Calculate the speeds in the coordinates system defined by the april tag
      // It is important that clipping occurs here and not below as clipping in the field coordinates 
      // will lead to paths that may initially veer away from the target.
      double xSpeedAprilTag = xController.calculate(aprilTagErrorVector.getX(), 0);
      double ySpeedAprilTag = yController.calculate(aprilTagErrorVector.getY(), 0);

      SmartDashboard.putNumber("GAAX", xSpeedAprilTag);
      SmartDashboard.putNumber("GAAY", ySpeedAprilTag);
      
      // Enforce maThrottle by scaling the magnitude of the error vector.
      // If we clamped instead we would see a slightly off initial angle when clipping happens.
      if (Math.sqrt(xSpeedAprilTag*xSpeedAprilTag + ySpeedAprilTag*ySpeedAprilTag) > config.getMaxThrottle()) {
        double scaleFactor =  config.getMaxThrottle() / Math.sqrt(xSpeedAprilTag*xSpeedAprilTag + ySpeedAprilTag*ySpeedAprilTag);
        xSpeedAprilTag *= scaleFactor;
        ySpeedAprilTag *= scaleFactor;
      }

      // Rotate the calculated speeds back to field coordinates.
      Translation2d aprilSpeedsPose = new Translation2d(xSpeedAprilTag,ySpeedAprilTag);
      Translation2d unrotatedSpeedsPose = aprilSpeedsPose.rotateBy(targetPose.getRotation().times(-1));
      
      // Use the unrotated speeds to control the robot.  It is possible that these speeds could exceed te max throttle but dont
      // clip them unless absolutely necessary to avoid artifacts in the paths.
      // The extra magnitude is more or less limited to sqrt(2)*maxThrottle
      xSpeed = MathUtil.clamp(unrotatedSpeedsPose.getX(), -1.0, 1.0);
      ySpeed = MathUtil.clamp(unrotatedSpeedsPose.getY(), -1.0, 1.0);

      // We are controlling rotation whether we use it for "onTarget" calculations or not.
      rotationSpeed = MathUtil.clamp(rotationController.calculate(rotationError, 0),-config.getMaxRotation(), config.getMaxRotation());
    }
    if (debug) SmartDashboard.putNumber("G2A xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("G2A ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("G2A rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);

    // For debug, update the forced robot location based on the x,y,theta applied.
    if (simulatingRobot) {
      double scale=0.02; // Rate that the simulation applies the changes to the robot's position.
      simulatedRobotPose = new Pose2d(simulatedRobotPose.getX()-xSpeed*scale, simulatedRobotPose.getY()-ySpeed*scale, new Rotation2d(desiredRotation)); //simulatedRobotPose.getRotation().minus(new Rotation2d(rotationSpeed*scale)));
    }
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
} // GotoAprilTagCommand class
