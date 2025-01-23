// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utilities;
import frc.robot.subsystems.DriveSubsystemSRX;

/**
 * This comand drives the robot the specified distance
 * If controlRotation is set, it will try to hold the angle in the move pose.
 * If controlRotation is set, it will just manage the X,Y of the move.
 */
public class DriveRobotCommand extends Command {

  public class DriveRobotConfig {
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
    public DriveRobotConfig() {
      maxThrottle = 0.20;
      linearP = 2.0; // 03-07-2024 was 2.7, changed to 2.0
      linearI = 0.0; // linearP/100.0;
      linearD = 0.0; // linearP*10.0;
      linearF = 0.0;
      linearIZone = Units.inchesToMeters(4.0);
      linearTolerance = Units.inchesToMeters(2.0);

      maxRotation = 0.8;
      angularP =  0.35;
      angularI = 0.0; // angularI/100.0;
      angularD = 0.0; // angularP*10.0;
      angularF = 0.0;
      angularIZone = Units.degreesToRadians(10.0);
      angularTolerance = Units.degreesToRadians(5.0);
    }
    public DriveRobotConfig setMaxThrottle(double maxThrottle) {this.maxThrottle = maxThrottle; return this; };
    public DriveRobotConfig setLinearP(double linearP) {this.linearP = linearP; return this; };
    public DriveRobotConfig setLinearI(double linearI) {this.linearI = linearI; return this; };
    public DriveRobotConfig setLinearD(double linearD) {this.linearD = linearD; return this; };
    public DriveRobotConfig setLinearF(double linearF) {this.linearF = linearF; return this; };
    public DriveRobotConfig setLinearIZone(double linearIZone) {this.linearIZone = linearIZone; return this; };
    public DriveRobotConfig setLinearTolerance(double linearTolerance) {this.linearTolerance = linearTolerance; return this; };

    public DriveRobotConfig setMaxRotation(double maxRotation) {this.maxRotation = maxRotation; return this; };
    public DriveRobotConfig setAngularP(double angularP) {this.angularP = angularP; return this; };
    public DriveRobotConfig setAngularI(double angularI) {this.angularI = angularI; return this; };
    public DriveRobotConfig setAngularD(double angularD) {this.angularD = angularD; return this; };
    public DriveRobotConfig setAngularF(double angularF) {this.angularF = angularF; return this; };
    public DriveRobotConfig setAngularIZone(double angularIZone) {this.angularIZone = angularIZone; return this; };
    public DriveRobotConfig setAngularTolerance(double angularTolerance) {this.angularTolerance = angularTolerance; return this; };

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
  } // DriveRobotConfig Class

  private final DriveSubsystemSRX robotDrive;
  private final Pose2d relativeMove;
  private final boolean controlRotation;
  private final DriveRobotConfig config;
  private Pose2d startingPose;
  private Pose2d targetPose;
  private PIDController xController;
  private PIDController yController;
  private PIDController rotationController;
  private boolean onTarget;
  private boolean debug = false;
  
  /** Creates a new DriveDistanceCommand. */
  public DriveRobotCommand(DriveSubsystemSRX robotDrive, Pose2d relativeMove, boolean controlRotation) {
    this.robotDrive = robotDrive;
    this.relativeMove = relativeMove;
    this.controlRotation = controlRotation;
    this.config = new DriveRobotConfig();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    double linearP = config.getLinearP();
    double linearI = config.getLinearI();

    // get the robot's current pose from the drivetrain
    startingPose = robotDrive.getPose();
    // add the relativeMove to the startingPose // There is an alliance component to this.   
    // I'm assuming the caller has handled it in the relativeMove.
    if (Utilities.isBlueAlliance()) {
    targetPose = new Pose2d(
      startingPose.getX() + relativeMove.getX(),
      startingPose.getY() + relativeMove.getY(),
      startingPose.getRotation().rotateBy(relativeMove.getRotation()));
      if (debug) SmartDashboard.putString("DR all", "Blue");

    } else if (Utilities.isRedAlliance()) {
      // This just inverts the X move as the field this year is rotated about the center of the field.
      targetPose = new Pose2d(
      startingPose.getX() - relativeMove.getX(),
      startingPose.getY() - relativeMove.getY(),
      startingPose.getRotation().rotateBy(relativeMove.getRotation().rotateBy(new Rotation2d(Math.PI))));
      if (debug) SmartDashboard.putString("DR all", "Red");

    } else {
      targetPose = startingPose; // Do nothing if we dont have an alliance.
      if (debug) SmartDashboard.putString("DR all", "None");

    }
    xController = new PIDController(linearP, linearI, config.getLinearD());
    xController.setIZone(0.1); // This is meters so about 4 inches  // TODO Needs tuning.
    yController = new PIDController(linearP, linearI, config.getLinearD());
    yController.setIZone(0.1); // NEW 2/1/2024 // TODO Needs Tuning.
    if (controlRotation) {
      rotationController = new PIDController(config.getAngularP(), config.getAngularI(), config.getAngularD());
      rotationController.setTolerance(1.0); // did not work, dont understand yet
      rotationController.enableContinuousInput(-Math.PI, Math.PI); // Tell PID Controller to expect inputs between -180 and 180 degrees (in Radians). // NEW 2/1/2024
    }
    onTarget = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translationError;
    double rotationError = 0.0;
    Pose2d currentPose;
    double xSpeed = 0.0;
    double ySpeed = 0.0;
    double rotationSpeed = 0.0;
    currentPose = robotDrive.getPose();
    if (debug) Utilities.toSmartDashboard("Drive Pose", currentPose);
    if (debug) Utilities.toSmartDashboard("Drive target", targetPose);
    // Calculate the X and Y and rotation offsets to the target location
    translationError = new Translation2d( targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
    // Calculate the difference in rotation between the PoseEstimator and the TargetPose
    // Make sure the rotation error is between -PI and PI
    if (controlRotation)
      rotationError = MathUtil.inputModulus(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians(), -Math.PI, Math.PI);
      if (debug) SmartDashboard.putNumber("Drive R Error", Units.radiansToDegrees(rotationError));
      if (debug) SmartDashboard.putNumber("Drive X Error", translationError.getX());
      if (debug) SmartDashboard.putNumber("Drive Y Error", translationError.getY());
    
    // Test to see if we have arrived at the requested pose within the specified toleranes
    if (Math.abs(translationError.getX()) < config.getLinearTolerance()
    &&  Math.abs(translationError.getY()) < config.getLinearTolerance()
    &&  Math.abs(rotationError) < config.getAngularTolerance()) {
      // Yes, we have arrived
      xSpeed = 0.0;
      ySpeed = 0.0;
      rotationSpeed = 0.0;
      onTarget = true;
    } else {
      // TODO fine tune PID Controllers and max speeds      
      xSpeed = MathUtil.clamp(xController.calculate(translationError.getX(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      ySpeed = MathUtil.clamp(yController.calculate(translationError.getY(), 0), -config.getMaxThrottle(), config.getMaxThrottle());
      if (controlRotation)
        rotationSpeed = -MathUtil.clamp(-rotationController.calculate(rotationError, 0),-config.getMaxRotation(), config.getMaxRotation()); // TODO Check sign  & Clean up 3 negations :-)
      else
        rotationSpeed = 0.0;
      onTarget = false;
    }
    if (debug) SmartDashboard.putBoolean("Drive OnTarget", onTarget);

    if (debug) SmartDashboard.putNumber("Drive xSpeed", xSpeed);
    if (debug) SmartDashboard.putNumber("Drive ySpeed", ySpeed);
    if (debug) SmartDashboard.putNumber("Drive rSpeed", rotationSpeed);
    robotDrive.drive(-xSpeed, -ySpeed, -rotationSpeed, true, true);
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
}
