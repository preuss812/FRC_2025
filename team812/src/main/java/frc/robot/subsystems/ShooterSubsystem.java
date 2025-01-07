// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;



public class ShooterSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_shooter = new WPI_TalonSRX(CANConstants.kShooterMotor);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    m_shooter.configFactoryDefault();
    m_shooter.setNeutralMode(NeutralMode.Brake);

    // This is a CLOSED loop system. Do not uncomment or enable
    // OpenloopRamp for the PID controlled shooter.
    // m_shooter.configOpenloopRamp(PidConstants.xxx_kShooter_rampRate_xxx);

    // Invert motor (setInverted) so that the Talon LEDs are green when driving
    // forward (up)
    // Phase sensor should have a positive increment as the Talon drives the shooter up
    m_shooter.setInverted(true);
    m_shooter.setSensorPhase(true); // Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_shooter.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_shooter.configNominalOutputForward(0, 10);
    m_shooter.configNominalOutputReverse(0, 10);
    m_shooter.configPeakOutputForward(0.6, 10);
    m_shooter.configPeakOutputReverse(-0.6, 10);

    // Configure the Motion Magic parameters for PID 0 within the Talon
    // The values for P, I, D, and F will need to be determined emperically
    m_shooter.selectProfileSlot(0, 0);
    m_shooter.config_kP(0, PidConstants.kShooter_kP, 10);
    m_shooter.config_kI(0, PidConstants.kShooter_kI, 10);
    m_shooter.config_kD(0, PidConstants.kShooter_kD, 10);
    m_shooter.config_kF(0, PidConstants.kShooter_kF, 10);

    // Velocity in sensor units per 100ms
    m_shooter.configMotionCruiseVelocity(150.0, 10);
    // Acceleration in sensor units per 100ms per second
    m_shooter.configMotionAcceleration(150.0, 10);

    // Make sure the forward and reverse limit switches are enabled and configured
    // normally open
    m_shooter.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_shooter.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_shooter.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

  }
  
  public void stop() {
    m_shooter.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, ShooterConstants.kUnshootSpeed,ShooterConstants.kShootSpeed);
    m_shooter.set(ControlMode.PercentOutput, clampedSpeed);
  }
  public void intake() {
    m_shooter.set(ControlMode.PercentOutput,ShooterConstants.kIntakeSpeed);
  } 

  public void shoot() {
    m_shooter.set(ControlMode.PercentOutput,ShooterConstants.kShootSpeed);
  } 

  public void unshoot() {
    m_shooter.set(ControlMode.PercentOutput, ShooterConstants.kUnshootSpeed);
  }

  @Override
  public void periodic() {
  }
}
