// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.photonvision.PhotonCamera;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;  // To access the Black Box Controller.
import frc.robot.Utilities;

public class DriveOnAprilTagProjectionCommand extends Command {
  
  public class DriveOnAprilTagProjectionConfig {
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
    public DriveOnAprilTagProjectionConfig() {
      maxThrottle = 0.70;
      linearP = 2.0;
      linearI = 0.0; // linearP/100.0;
      linearD = 0.0; // linearP*10.0;
      linearF = 0.0;
      linearIZone = Units.inchesToMeters(4.0);
      linearTolerance = Units.inchesToMeters(2.0);

      maxRotation = 0.8;
      angularP =  0.35;
      angularI = 0.0; // angularI/100.0;
      angularD = 0.0; //angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(5.0);
    }
    public DriveOnAprilTagProjectionConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public DriveOnAprilTagProjectionConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public DriveOnAprilTagProjectionConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public DriveOnAprilTagProjectionConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public DriveOnAprilTagProjectionConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public DriveOnAprilTagProjectionConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public DriveOnAprilTagProjectionConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public DriveOnAprilTagProjectionConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public DriveOnAprilTagProjectionConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public DriveOnAprilTagProjectionConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public DriveOnAprilTagProjectionConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public DriveOnAprilTagProjectionConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public DriveOnAprilTagProjectionConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public DriveOnAprilTagProjectionConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

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
  } // DriveOnAprilTagProjectionConfig Class

  /** 
   * Creates a new command to move the robot along the line that projects perpendicularly from an april tag.
   */
  private final PoseEstimatorSubsystem poseEstimatorSubsystem;
  private final DriveSubsystemSRX robotDrive;
  private final PhotonCamera photonCamera;
  private final XboxController xbox;
  private final DriveOnAprilTagProjectionConfig config;

  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private static Pose2d tagPose = null;
  private static boolean commandIsActive = false;
  
  private double p; // The constant P from the normal form for the april tag projection line.
  private double xSign; // The sign of x axis motion toward the april tag.
  private double ySign; // The sign of y axis motion toward the april tag. 
  private double projectionM;
  private double projectionB;

  private boolean debug = true;
  private static double lastTheta;

  /**
   * Drive to the specified distance from the best april tag currently in view.
   * @params poseEstimatorSubsystem - the pose estimator subsystem
   * @params robotDrive - the DriveSubsystemSRX drivetrain to drive the robot
   * @params photonCamera - the Photon Camera Subsystem used to see the April Tags.
   */
  public DriveOnAprilTagProjectionCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , XboxController xbox) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.xbox = xbox;
    this.config = new DriveOnAprilTagProjectionConfig();
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  public DriveOnAprilTagProjectionCommand(PoseEstimatorSubsystem poseEstimatorSubsystem
    , DriveSubsystemSRX robotDrive
    , PhotonCamera photonCamera
    , XboxController xbox
    , DriveOnAprilTagProjectionConfig config) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    this.robotDrive = robotDrive;
    this.photonCamera = photonCamera;
    this.xbox = xbox;
    this.config = config;
    addRequirements(robotDrive, poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("DP","Active");
    tagPose = null;
    commandIsActive = false;
    p = 0.0;
    xSign = 0.0;
    ySign = 0.0;
    //debugPID = RobotContainer.m_BlackBox.isSwitchCenter();
    double linearP = 0; // If no tag is found, do not energize the motors.
    double linearI = 0;

    var pipelineResult = photonCamera.getLatestResult();
    if (pipelineResult.hasTargets()) {
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      SmartDashboard.putNumber("DA FID", fiducialId);

      // Get the tag pose from field layout - consider that the layout will be null if it failed to load
      tagPose = poseEstimatorSubsystem.getAprilTagPose(fiducialId);
      // Re-align the robot drive and pose subsystem.
      //robotDrive.resetOdometry(poseEstimatorSubsystem.getCurrentPose());

      if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0) {
        commandIsActive = true;
        linearP = config.getLinearP();
        linearI = config.getLinearI();
        Utilities.toSmartDashboard("DA tag",tagPose);
  
        // Compute the projected line from the april tag.
        // I'm using the "normal form" of the line.
        double theta = tagPose.getRotation().getRadians();
        SmartDashboard.putNumber("DP theta", theta);
        SmartDashboard.putNumber("DP cos", Math.cos(theta));

        p = tagPose.getX()* Math.cos(theta) + tagPose.getY() * Math.sin(theta);
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
          projectionB = tagPose.getY() - projectionM * tagPose.getX();
        }
        SmartDashboard.putNumber("DP x", xSign);
        SmartDashboard.putNumber("DP y", ySign);
      }
    } else {
      tagPose = null;
      commandIsActive = false;
    }
    SmartDashboard.putNumber("DA P", linearP);
    SmartDashboard.putNumber("DA I", linearI);
    // Create PID controllers for x, y, rotation.
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
    lastTheta = 4.0;
    rotationController.setTolerance(config.getAngularTolerance()); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationErrorToTarget;
    Pose2d currentPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    Translation2d nominalMove;
    Translation2d move;
    Translation2d uncorrectedGoal;
    Translation2d correctedGoal;
    double throttle;

    if (tagPose != null) {
      throttle = -xbox.getRightY();
      rotationSpeed = MathUtil.applyDeadband(xbox.getRightX(), OIConstants.kDriveDeadband*2); // Based on xbox right joystick / ignore calculations above.
      
      SmartDashboard.putNumber("DA throttle", throttle);
      //throttle = 0.5;
      currentPose = poseEstimatorSubsystem.getCurrentPose();
      nominalMove = new Translation2d(throttle*config.getLinearP()/1.0, 0.0);
      move = nominalMove.rotateBy(tagPose.getRotation().plus(new Rotation2d(Math.PI))); // Rotate the nominal move to the tag pose.
      uncorrectedGoal = currentPose.getTranslation().plus(move);
      if (debug) SmartDashboard.putString("DA Move",move.toString());
      if (debug) SmartDashboard.putString("DA unc",uncorrectedGoal.toString());
      
      if (Math.abs(rotationSpeed) < 0.05 && lastTheta != 4.0) {
        double deltaRotation = MathUtil.inputModulus(currentPose.getRotation().getRadians() - lastTheta, -Math.PI, Math.PI);
        rotationSpeed = -MathUtil.clamp(
          rotationController.calculate(deltaRotation,0),
            -config.getMaxRotation(),
            config.getMaxRotation());
      } else {
        lastTheta = currentPose.getRotation().getRadians();
      }

      // We now have the goal position ignoring the distance from the april tag projection
      // Compute a correction factor to keep us on the projection line.
      if (Math.abs(throttle) <= 0.05) {
        correctedGoal = currentPose.getTranslation();  // has the effect of pid producting x,y speeds of 0.0.
      } else if (xSign == 0) {
        // The line is vertical.
        // The correction is simple set the X goal to be the X coordinate of the goal line.
        correctedGoal = new Translation2d( tagPose.getX(), uncorrectedGoal.getY());
      } else if (ySign == 0) {
        // The line is horizontal
        // The correction is simple set the Y goal to be the Y coordinate of the goal line.
        correctedGoal = new Translation2d(uncorrectedGoal.getX(), tagPose.getY());
      } else {
        // Intersect the 2 lines to find the point to be on the projection line.
        double perpendicularM = 1/projectionM;
        double perpendicularB = uncorrectedGoal.getY() - projectionM * uncorrectedGoal.getX();
        double intersectX = (perpendicularB - projectionB)/(projectionM - perpendicularM);
        double intersectY = projectionM*intersectX + projectionB;
        correctedGoal = new Translation2d(intersectX, intersectY);
      }

      if (debug) SmartDashboard.putString("DA goal", correctedGoal.toString());

      // Calculate the X and Y and rotation offsets to the target location
      translationErrorToTarget = new Translation2d( correctedGoal.getX() - currentPose.getX(), correctedGoal.getY() - currentPose.getY());
      
      if (debug) SmartDashboard.putNumber("DA X", translationErrorToTarget.getX());
      if (debug) SmartDashboard.putNumber("DA Y", translationErrorToTarget.getY());
      
      xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTarget.getX(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTarget.getY(), 0), -config.getMaxThrottle(), config.getMaxThrottle());

    }
    if (debug) SmartDashboard.putNumber("DA xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("DA ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("DA rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.drive(0, 0, 0, true, true);
        SmartDashboard.putString("DP","Done");
    commandIsActive = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (tagPose == null);
  }

  /**
   * Set the robot's rotation relative to the april tag being use for driving.
   * @param angleToTag - the angle in radians the robot should face relative to the april tag 0=facing the tag.
   */
  public static void setAngle(double angleToTag) {
    if (commandIsActive && tagPose != null) {
      lastTheta = MathUtil.inputModulus(angleToTag + tagPose.getRotation().getRadians() + Math.PI, -Math.PI, Math.PI);
    }
  }

  public static boolean isActive() {
    return commandIsActive;
  }
} // DriveOnAprilTagProjectionCommand class
