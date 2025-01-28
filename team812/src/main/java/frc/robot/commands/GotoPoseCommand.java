// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystemSRX;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.Utilities;

public class GotoPoseCommand extends Command {

  public class GotoPoseConfig {
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
    public GotoPoseConfig() {
      maxThrottle = 0.20;
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
    public GotoPoseConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public GotoPoseConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public GotoPoseConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public GotoPoseConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public GotoPoseConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public GotoPoseConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public GotoPoseConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public GotoPoseConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public GotoPoseConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public GotoPoseConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public GotoPoseConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public GotoPoseConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public GotoPoseConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public GotoPoseConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

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
  } // GotoPoseConfig Class

  /** Creates a new command to move the robot to the specified pose. */
  protected final PoseEstimatorSubsystem m_PoseEstimatorSubsystem;
  protected final DriveSubsystemSRX m_DriveSubsystemSRXSubsystem;
  protected final Pose2d m_targetPose;
  protected final GotoPoseConfig m_config;
  protected Pose2d targetPose;

  protected PIDController xController;
  protected PIDController yController;
  protected PIDController rotationController;
  protected boolean onTarget;
  private boolean debug = false;
  
  public GotoPoseCommand(PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DriveSubsystemSRX DriveSubsystemSRXSubsystem
    , double targetX
    , double targetY
    , double targetRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_PoseEstimatorSubsystem = PoseEstimatorSubsystem;
    m_DriveSubsystemSRXSubsystem = DriveSubsystemSRXSubsystem;
    m_targetPose = new Pose2d(targetX, targetY, new Rotation2d(targetRotation));
    onTarget = false;
    m_config = new GotoPoseConfig();
    addRequirements(PoseEstimatorSubsystem, DriveSubsystemSRXSubsystem);
  }

  public GotoPoseCommand(PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DriveSubsystemSRX DriveSubsystemSRXSubsystem
    , Pose2d targetPose) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    m_PoseEstimatorSubsystem = PoseEstimatorSubsystem;
    m_DriveSubsystemSRXSubsystem = DriveSubsystemSRXSubsystem;
    m_targetPose = targetPose;
    onTarget = false;
    m_config = new GotoPoseConfig();
    addRequirements(PoseEstimatorSubsystem, DriveSubsystemSRXSubsystem);
    
  }

  public GotoPoseCommand(PoseEstimatorSubsystem PoseEstimatorSubsystem
    , DriveSubsystemSRX DriveSubsystemSRXSubsystem
    , Pose2d targetPose
    , GotoPoseConfig config) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    m_PoseEstimatorSubsystem = PoseEstimatorSubsystem;
    m_DriveSubsystemSRXSubsystem = DriveSubsystemSRXSubsystem;
    m_targetPose = targetPose;
    onTarget = false;
    this.m_config = config;
    addRequirements(PoseEstimatorSubsystem, DriveSubsystemSRXSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //debugPID = RobotContainer.m_BlackBox.isSwitchCenter();
    double linearP = m_config.getLinearP();
    double linearI = m_config.getLinearI();
    
    xController = new PIDController(
      linearP, // m_config.getLinearP(),
      linearI, // m_config.getLinearI(),
      m_config.getLinearD()
    );
    xController.setIZone(m_config.getLinearIZone());
    yController = new PIDController(
      linearP, // m_config.getLinearP(),
      linearI, // m_config.getLinearI(),
      m_config.getLinearD()
    );
    yController.setIZone(m_config.getAngularIZone()); // TODO Needs Tuning.

    rotationController = new PIDController(
      m_config.getAngularP(),
      m_config.getAngularI(),
      m_config.getAngularD()
      );
    rotationController.setTolerance(m_config.getAngularTolerance()); // did not work, dont understand yet
    rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians).
    onTarget = false;
    targetPose = m_targetPose;
    if (debug) Utilities.toSmartDashboard("GotoTarget", targetPose);
    if (debug) SmartDashboard.putBoolean("GotoPoseOnTarget", false); // We will need to check in execute
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean correctRotation = true; // TODO test with this = false;
    Translation2d translationErrorToTarget;
    Translation2d translationErrorToTargetCorrectedForRotation;
    double rotationError;
    Rotation2d rotationErrorEstimationToDriveTrain;
    Pose2d estimatedPose;
    Pose2d driveTrainPose;
    double estimatedRotationToDriveTrainRotation;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;

    estimatedPose = m_PoseEstimatorSubsystem.getCurrentPose();
    if (debug) Utilities.toSmartDashboard("GotoPose Pose", estimatedPose);
    // Calculate the X and Y and rotation offsets to the target location
    translationErrorToTarget = new Translation2d( targetPose.getX() - estimatedPose.getX(), targetPose.getY() - estimatedPose.getY());
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    rotationError = MathUtil.inputModulus(targetPose.getRotation().getRadians() - estimatedPose.getRotation().getRadians(), -Math.PI, Math.PI);
    if (debug) SmartDashboard.putNumber("GotoPose RError", Units.radiansToDegrees(rotationError));
    if (debug) SmartDashboard.putNumber("GotoPoseXOffset", translationErrorToTarget.getX());
    if (debug) SmartDashboard.putNumber("GotoPoseYOffset", translationErrorToTarget.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationErrorToTarget.getX()) < m_config.getLinearTolerance()
    &&  Math.abs(translationErrorToTarget.getY()) < m_config.getLinearTolerance()
    &&  Math.abs(rotationError) < m_config.getAngularTolerance()) {
      // Yes, we have arrived
      if (debug) SmartDashboard.putBoolean("GotoPoseOnTarget", true);
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // Rotate the drive X and Y taking into account the difference in the coordinates
      // between the DriveTrain and the PoseEstimator.
      // Calculate the difference in rotation between the PoseEstimator and the DriveTrainPose
      if (correctRotation) {
        driveTrainPose = m_DriveSubsystemSRXSubsystem.getPose();
        estimatedRotationToDriveTrainRotation = estimatedPose.getRotation().getRadians() - driveTrainPose.getRotation().getRadians();
        estimatedRotationToDriveTrainRotation = MathUtil.inputModulus(estimatedRotationToDriveTrainRotation, 0.0, Math.PI*2.0);
        
        if (debug) SmartDashboard.putNumber("RotationError", estimatedRotationToDriveTrainRotation);

        rotationErrorEstimationToDriveTrain = new Rotation2d(estimatedRotationToDriveTrainRotation);
        translationErrorToTargetCorrectedForRotation = translationErrorToTarget.rotateBy(rotationErrorEstimationToDriveTrain);    // TODO Check sign of rotation.
        xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTargetCorrectedForRotation.getX(), 0), -m_config.getMaxThrottle(), m_config.getMaxThrottle());
        ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTargetCorrectedForRotation.getY(), 0), -m_config.getMaxThrottle(), m_config.getMaxThrottle());
      } else {
        xSpeed = MathUtil.clamp(xController.calculate(translationErrorToTarget.getX(), 0), -m_config.getMaxThrottle(), m_config.getMaxThrottle());
        ySpeed = MathUtil.clamp(yController.calculate(translationErrorToTarget.getY(), 0), -m_config.getMaxThrottle(), m_config.getMaxThrottle());
      }
      
      rotationSpeed = -MathUtil.clamp(
        -rotationController.calculate(rotationError, 0.0)
        ,-m_config.getMaxRotation()
        , m_config.getMaxRotation()
      ); // TODO Clean up double negation
    }
    if (debug) SmartDashboard.putNumber("GotoPose xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("GotoPose ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("GotoPose rSpeed", rotationSpeed);
    m_DriveSubsystemSRXSubsystem.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystemSRXSubsystem.drive(0, 0, 0, true, true); // TODO Verify signs of inputs 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget;
  }
}
