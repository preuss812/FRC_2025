// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
//import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.configs.CANcoderConfigurator;
//import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.sim.CANcoderSimState;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.CANConstants;

public class MAXSRXSwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final CANcoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private PIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private boolean debug = false;

  /**
   * Constructs a MAXSRXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSRXSwerveModule(int drivingCANId, int turningCANId, int turningEncoderCANId, double chassisAngularOffset) {

    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
    //m_turningPIDController = new PIDController(0.5,0.0,0.1);
    m_turningPIDController = new PIDController(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = new CANcoder(turningEncoderCANId);

    /* User can change the configs if they want, or leave it empty for factory-default */

    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    // Using PIDController instead: m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    // Using PIDController instead: m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    //m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    //m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
    //m_turningEncoder.configFeedbackCoefficient(0.0015339794,"radians",SensorTimeBase.PerSecond);
    // Seems like a lot of the builtin conversions for units were removed in Phoenix5 -> Phoenix6.
    // I will convert the units below when we get the position.

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    // Had to handle below: m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    //m_turningPIDController.setPositionPIDWrappingEnabled(true);
    //m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    //m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
    m_turningPIDController.enableContinuousInput(ModuleConstants.kTurningEncoderPositionPIDMinInput,ModuleConstants.kTurningSRXEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    // No Feed Forward: m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    // No range option, clamp later instead: m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
    //    ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // These next two lines did nothing!!!
    //m_drivingSparkMax.setClosedLoopRampRate(10.0);
    //m_drivingSparkMax.setOpenLoopRampRate(10.0);
    
    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_turningEncoder.setPosition(m_turningEncoder.getAbsolutePosition().getValue()); // An attempt to make the position == the absolute position
    m_desiredState.angle = new Rotation2d(this.CANCoderPositionRadians()); // Set desired angle to current angle so it wont move.

    m_drivingEncoder.setPosition(0);
    
  }
  /**
   * Helper function that converts CANCoder output into an angle in radians
   * in the range of 0 to 2*pi.
   * The CANCoder returns values in units of rotation, ie 1.0 = 1 full roration counterclockwise from the 0.0 position.
   * @return The current turning direction in radians
   */
  public double CANCoderPositionRadians() {
    double CANCoderPosition;
    CANCoderPosition = m_turningEncoder.getPosition().getValue();

    /**
     * Since we only care about the direction the wheel is pointing and not about how many times it has
     * rotated away from the origin (= 0.0 == Straight ahead), make sure the value is in the range of 0.0
     * to 1.0.  The CANCoder will count the number of rotations and that would just confuse our
     * interpretation of the direction.
     */
    CANCoderPosition = CANCoderPosition % 1.0;
     // Make sure the value is in the range of 00.0 to 1.0
     if (CANCoderPosition < 0.0) CANCoderPosition += 1.0;
    CANCoderPosition *= 2.0 * Math.PI; // Scale the the rotations into units of Radians.
    // Some debug that should eventually be removed:
    if (debug) SmartDashboard.putNumber("Absolute position"+m_turningEncoder.getDeviceID(), m_turningEncoder.getAbsolutePosition().getValue());
    if (debug && (m_turningEncoder.getDeviceID() == CANConstants.kSwerveLeftFrontCANCoder)) {
      SmartDashboard.putNumber("lf_angle_radians", CANCoderPosition);
      SmartDashboard.putNumber("lf_angle_degrees", CANCoderPosition/(2*Math.PI)*360.0);
    }
    return CANCoderPosition;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    double turningEncoderAngleRadians;
    turningEncoderAngleRadians = this.CANCoderPositionRadians();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoderAngleRadians - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    double turningEncoderAngleRadians;
    turningEncoderAngleRadians = this.CANCoderPositionRadians();
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(turningEncoderAngleRadians - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    double currentTurningAngleInRadians = this.CANCoderPositionRadians();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(currentTurningAngleInRadians));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    m_turningPIDController.setSetpoint(optimizedDesiredState.angle.getRadians());
    
    double pidOutput = m_turningPIDController.calculate(currentTurningAngleInRadians);
    m_turningSparkMax.set(pidOutput);

    if (debug && (m_turningEncoder.getDeviceID() == CANConstants.kSwerveLeftFrontCANCoder)) {
      SmartDashboard.putNumber("optimizedTurn", optimizedDesiredState.angle.getRadians());
      SmartDashboard.putNumber("optimizedTurnError", optimizedDesiredState.angle.getRadians()-currentTurningAngleInRadians);
      SmartDashboard.putNumber("pidOutput", pidOutput);
    }

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public void zeroOrientation() {
    
  }
  public void quiesce() {
    setDesiredState(m_desiredState); // Tell the module to want to be where it already is.
  }
}
