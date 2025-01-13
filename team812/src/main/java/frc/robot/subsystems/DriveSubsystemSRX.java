// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ModifiedSlewRateLimiter;
import frc.robot.Utilities;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

public class DriveSubsystemSRX extends SubsystemBase {
  // Create MAXSRXSwerveModules
  private final MAXSRXSwerveModule m_frontLeft = new MAXSRXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftTurningEncoderCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSRXSwerveModule m_frontRight = new MAXSRXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightTurningEncoderCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSRXSwerveModule m_rearLeft = new MAXSRXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftTurningEncoderCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSRXSwerveModule m_rearRight = new MAXSRXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightTurningEncoderCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB1);


  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private ModifiedSlewRateLimiter m_magLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRate, DriveConstants.kMagnitudeDecreaseSlewRate, 0);
  private ModifiedSlewRateLimiter m_rotLimiter = new ModifiedSlewRateLimiter(DriveConstants.kRotationalIncreaseSlewRate, DriveConstants.kRotationalDecreaseSlewRate, 0);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  public enum DrivingMode {
    SPEED,
    PRECISION
  }
  // Default driving mode and associated constants to SPEED mode.
  private DrivingMode drivingMode = DrivingMode.SPEED; // Default to full speed driving mode.
  private double maxSpeedMetersPerSecond    = DriveConstants.kMaxSpeedMetersPerSecond;
  private double maxAngularSpeed            = DriveConstants.kMaxAngularSpeed;
  private double directionSlewRate          = DriveConstants.kDirectionSlewRate;
  //private double magnitudeIncreaseSlewRate  = DriveConstants.kMagnitudeIncreaseSlewRate;
  //private double magnitudeDecreaseSlewRate  = DriveConstants.kMagnitudeDecreaseSlewRate;
  //private double rotationalIncreaseSlewRate = DriveConstants.kRotationalIncreaseSlewRate;
  //private double rotationalDecreaseSlewRate = DriveConstants.kRotationalDecreaseSlewRate;
  private final boolean debug = true;

  /** Creates a new DriveSubsystemSRXSRX. */
  public DriveSubsystemSRX() {
    setDrivingMode(DrivingMode.SPEED);

    // TODO Do we need to reset the gyro here?
  }
// TODO: This seems redundant to the code below in periodic.  Perhaps should refactor.
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        };
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    if (debug) {
      SmartDashboard.putNumber("gyro_angle", -m_gyro.getAngle());
      //SmartDashboard.putNumber("gyro_offset", this.m_odometry.I want the angle offset but it's not public);
      Utilities.toSmartDashboard("DriveTrain", this.getPose()); 
      SmartDashboard.putNumber("Robot X", this.getPose().getX()); 
      SmartDashboard.putNumber("Robot Y", this.getPose().getY()); 
      // SmartDashboard.putNumber("gyro_Xaccel", m_gyro.getAccelX());
      // SmartDashboard.putNumber("gyro_Yaccel", m_gyro.getAccelY());
      // SmartDashboard.putNumber("gyro_navX", m_ahrs.getAngle());
    }

    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
  * This slew rate limiter uses a different slew rate for increasing or descreasing velocity.
  * Form the 2024 robot we saw that it was easier for the robot drivers to control the robot
  * when the robot acceleration was ramped up more slowly to allow finer control of the robot.
  * However, drivers had trouble controlling the robot when the decelaration was ramped slowly.
  * That is what led us to the asymmetrical ramp rates.
  * Note that we are slewing the inputs to the drive motors, not the actual motor speeds.
  * Note also that we are using the nominal cycle time, 20ms, instead of the actual.
  * Note MAXSPEED is the maximum allowed speed of the robot in meters/sec.
  *
  * Parameters:
    @param last - the last output to the drive motors.  Must be between MAXSPEED and MAXSPEED as it represents the percent motor output.
    @param delta - the change to the output to the drive motors.  (last+delta) ust be between MAXSPEED and MAXSPEED as it represents the percent motor output.
    @param maxDecrease - the maximum decrease in output in percent/second.  Must be positive.  e.g. 0.5 indications 2.0 seconds to go from +/-MAXSPEED to 0.0.
    @param maxIncrease - the maximum increase in output in percent/second.  Must be positive.  e.g. 0.5 indications 2.0 seconds to go from 0.0 to +/-MAXSPEED.
    @param elapsedTime - the nominal time between calls to this method in seconds.  e.g. 0.02 for 50Hz.
                         This is used to convert the maxDecrease and maxIncrease to the actual change in output for this one periodic cycle.
  * @return the slew rate limited output  Must be between -MAXSPEED and MAXSPEED as it represents the desired robot speed in meters/sec.
  *
  * There is an edge case where the delta would cross through 0.  That would be a problem because we should shift from deceleration to acceleration
  * and use the different slew rate for the deceleration and acceleration regions.   Because the nominal period is 20ms we are ignoring that edge case.
  */
  public double slewRateLimiter(double last, double delta, double maxDecrease, double maxIncrease, double elapsedTime) {
    double next = last;
    int lastSign = (int)Math.signum(last);
    int deltaSign = (int)Math.signum(delta);

    // Branch on the sign of the last value
    // In retrospect it would be slightly less code to branch on the sign of the delta as when delta is 0,
    // we will carry over the last value regardless of what the last value was.
    switch(lastSign) {
      case -1:
        switch(deltaSign) {
          case -1:
            next = last + Math.max(delta, last - maxIncrease) * elapsedTime;
            break;
          case 0:
            next = last;
            break;
          case 1:
            next = last + Math.max(delta, last + maxDecrease) * elapsedTime;
            break;
          default:
            next = last; // Should never happen
            break;
        }
        break ;
      case 0:
        switch(deltaSign) {
          case -1:
            next = last + Math.max(delta, last - maxIncrease) * elapsedTime;
            break;
          case 0:
            next = last;
            break;
          case 1:
            next = last + Math.max(delta, last + maxIncrease) * elapsedTime;
            break;
          default:
            next = last; // Should never happen
            break;
        }
        break;
      case 1:
        switch(deltaSign) {
          case -1:
            next = last + Math.max(delta, last + maxDecrease) * elapsedTime;
            break;
          case 0:
            next = last;
            break;
          case 1:
            next = last + Math.max(delta, last + maxIncrease) * elapsedTime;
            break;
          default:
            next = last; // Should never happen
            break;
        }
        break;
      default:
        next = last; // Should never happen
        break;
    }

    return next;
  }
  public void allianceRelativeDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
      
      if (Utilities.isBlueAlliance())
        drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit); // The coordinates are fine if the 
      else
        drive(-xSpeed, -ySpeed, rot, fieldRelative, rateLimit); // Rotate the joystick inputs 180 degrees if we are on the Red Alliance.
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    boolean simpler2025SlewRateLimited = true;
    double xSpeedCommanded;
    double ySpeedCommanded;
    double nominalElapsedTime = 0.02; // 50Hz loop time

    if (debug) SmartDashboard.putNumber("DT x input", xSpeed);
    if (debug) SmartDashboard.putNumber("DT y input", ySpeed);

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        // This slows the rotation to reserve speed for translation.
        directionSlewRate = Math.abs(this.directionSlewRate / m_currentTranslationMag);
      } else {
        // If we are not moving in x,y, we can rotate as fast as we want.
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      
      if (debug) SmartDashboard.putNumber("directionSlewRate", directionSlewRate);

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      // Calculate the angle difference between the current and desired direction
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);  // Returns a number betwen -pi .. pi
      // if the change in angle is less than ~90 degrees, we can change the direction
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else if (simpler2025SlewRateLimited) {
      // TODO handle fieldRelative here:
      // Would make sense to just memorize the last x,y speed commanded.
      double lastXSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      double lastYSpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      double newXSpeedRequested = lastXSpeedCommanded + xSpeed * maxSpeedMetersPerSecond;
      double newYSpeedRequested = lastYSpeedCommanded + ySpeed * maxSpeedMetersPerSecond;
      double deltaXSpeedRequested = newXSpeedRequested - lastXSpeedCommanded;
      double deltaYSpeedRequested = newYSpeedRequested - lastYSpeedCommanded;

      xSpeedCommanded = slewRateLimiter(lastXSpeedCommanded, deltaXSpeedRequested, DriveConstants.kMagnitudeDecreaseSlewRate, DriveConstants.kMagnitudeIncreaseSlewRate*0.1, nominalElapsedTime);
      ySpeedCommanded = slewRateLimiter(lastYSpeedCommanded, deltaYSpeedRequested, DriveConstants.kMagnitudeDecreaseSlewRate, DriveConstants.kMagnitudeIncreaseSlewRate, nominalElapsedTime);
      m_currentRotation = rot;
    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * maxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * maxAngularSpeed;

    SmartDashboard.putNumber("DTxSpeed", xSpeed);
    SmartDashboard.putNumber("DTySpeed", ySpeed);
    SmartDashboard.putNumber("DTxSpeedDelivered", xSpeedDelivered);
    SmartDashboard.putNumber("DTySpeedDelivered", ySpeedDelivered);
    SmartDashboard.putNumber("DTrotDelivered", rot);
    
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    //SmartDashboard.putNumber("maxSpeedMetersPerSecond", maxSpeedMetersPerSecond);
    //SmartDashboard.putNumber("maxAngularSpeed", maxAngularSpeed);
    //SmartDashboard.putNumber("directionSlewRate",directionSlewRate); //          = DriveConstants.kDirectionSlewRate;
    //SmartDashboard.putNumber("magnitudeIncreaseSlewRate",magnitudeIncreaseSlewRate); //  = DriveConstants.kMagnitudeIncreaseSlewRate;
    //SmartDashboard.putNumber("magnitudeDecreaseSlewRate",magnitudeDecreaseSlewRate); //  = DriveConstants.kMagnitudeDecreaseSlewRate;
    //SmartDashboard.putNumber("rotationalIncreaseSlewRate",rotationalIncreaseSlewRate); // = DriveConstants.kRotationalIncreaseSlewRate;
    //SmartDashboard.putNumber("rotationalDecreaseSlewRate",rotationalDecreaseSlewRate); // = DriveConstants.kRotationalDecreaseSlewRate;
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the wheels to point straight ahead.
   */
  public void wheelsStraightAhead() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void wheels45() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the rotation of the robot.
   *
   * @return the robot's rotation as Rotation2d.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  /**
   * Reset the gyro angle as specified, presumably to align the drivetrain to the field.
   * @param desiredngle
   * @return
   */
  public double setAngleDegrees(double desiredAngle) {
    // There is something wrong here as the results are 180 out.
    if (debug) {
      SmartDashboard.putNumber("SetAngle", desiredAngle); // minus but we should have added m_gyro.inverted instead
      SmartDashboard.putNumber("SetAngleGyro", m_gyro.getAngle());
      SmartDashboard.putNumber("SetAngleOldAdj", m_gyro.getAngleAdjustment());
      SmartDashboard.putNumber("SetAngleNewAdj", desiredAngle - (m_gyro.getAngle() - m_gyro.getAngleAdjustment()));
    }
    m_gyro.setAngleAdjustment((desiredAngle - (m_gyro.getAngle() - m_gyro.getAngleAdjustment())));    
    if (debug) SmartDashboard.putNumber("SetAngleNew", m_gyro.getAngle());

    return m_gyro.getAngle();  // Return the new angle for chaining
  }

  public void quiesce() {
    m_frontLeft.quiesce();
    m_frontRight.quiesce();
    m_rearLeft.quiesce();
    m_rearRight.quiesce();
  }

  /**
   * setDrivingMode
   * @param drivingMode - SPEED or PRECISION
   * @return the previous drivingMode
   */
  public DrivingMode setDrivingMode(DrivingMode drivingMode) {
    DrivingMode result = this.drivingMode;
    this.drivingMode = drivingMode;
    if (drivingMode == DrivingMode.SPEED) {
      maxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecond;
      maxAngularSpeed = DriveConstants.kMaxAngularSpeed;
      directionSlewRate = DriveConstants.kDirectionSlewRate;
      //magnitudeIncreaseSlewRate = DriveConstants.kMagnitudeIncreaseSlewRate;
      //magnitudeDecreaseSlewRate = DriveConstants.kMagnitudeDecreaseSlewRate;
      //rotationalIncreaseSlewRate = DriveConstants.kRotationalIncreaseSlewRate;
      //rotationalDecreaseSlewRate = DriveConstants.kRotationalDecreaseSlewRate;
      m_magLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRate, DriveConstants.kMagnitudeDecreaseSlewRate, 0);
      m_rotLimiter = new ModifiedSlewRateLimiter(DriveConstants.kRotationalIncreaseSlewRate, DriveConstants.kRotationalDecreaseSlewRate, 0);
      SmartDashboard.putString("DriveMode", "SPEED");
    } else /* if (drivingMode == DrivingMode.PRECISION) */ {
      maxSpeedMetersPerSecond = DriveConstants.kMaxSpeedMetersPerSecondPM;
      maxAngularSpeed = DriveConstants.kMaxAngularSpeedPM;
      directionSlewRate = DriveConstants.kDirectionSlewRatePM;
      //magnitudeIncreaseSlewRate = DriveConstants.kMagnitudeIncreaseSlewRatePM;
      //magnitudeDecreaseSlewRate = DriveConstants.kMagnitudeDecreaseSlewRatePM;
      //rotationalIncreaseSlewRate = DriveConstants.kRotationalIncreaseSlewRatePM;
      //rotationalDecreaseSlewRate = DriveConstants.kRotationalDecreaseSlewRatePM;
      m_magLimiter = new ModifiedSlewRateLimiter(DriveConstants.kMagnitudeIncreaseSlewRatePM, DriveConstants.kMagnitudeDecreaseSlewRatePM, 0);
      m_rotLimiter = new ModifiedSlewRateLimiter(DriveConstants.kRotationalIncreaseSlewRatePM, DriveConstants.kRotationalDecreaseSlewRatePM, 0);

      SmartDashboard.putString("DriveMode", "PRECISION");
    }
    return result;
  }
  
  /**
   * getDrivingMode
   * @return the current drivingMode either SPEED or PRECISION
   */
  public DrivingMode getDrivingMode() {
    return drivingMode;
  }
  
}
