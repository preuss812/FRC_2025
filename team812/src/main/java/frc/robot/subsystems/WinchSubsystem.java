// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.WinchConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;



public class WinchSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_winch = new WPI_TalonSRX(CANConstants.kWinchMotor);
  //boolean endGame = false;

  /** Creates a new WinchSubsystem. */
  public WinchSubsystem() {
    m_winch.configFactoryDefault();
    m_winch.setNeutralMode(NeutralMode.Brake);

    // This is a CLOSED loop system. Do not uncomment or enable
    // OpenloopRamp for the PID controlled winch.
    // m_winch.configOpenloopRamp(PidConstants.xxx_kWinch_rampRate_xxx);

    // Invert motor (setInverted) so that the Talon LEDs are green when driving
    // forward (up)
    // Phase sensor should have a positive increment as the Talon drives the winch up
    m_winch.setInverted(true);
    m_winch.setSensorPhase(true); // Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_winch.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_winch.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_winch.configNominalOutputForward(0, 10);
    m_winch.configNominalOutputReverse(0, 10);
    m_winch.configPeakOutputForward(0.6, 10);
    m_winch.configPeakOutputReverse(-0.6, 10);

    // Configure the Motion Magic parameters for PID 0 within the Talon
    // The values for P, I, D, and F will need to be determined emperically
    m_winch.selectProfileSlot(0, 0);
    m_winch.config_kP(0, PidConstants.kWinch_kP, 10);
    m_winch.config_kI(0, PidConstants.kWinch_kI, 10);
    m_winch.config_kD(0, PidConstants.kWinch_kD, 10);
    m_winch.config_kF(0, PidConstants.kWinch_kF, 10);

    // Velocity in sensor units per 100ms
    m_winch.configMotionCruiseVelocity(150.0, 10);
    // Acceleration in sensor units per 100ms per second
    m_winch.configMotionAcceleration(150.0, 10);

    // Make sure the forward and reverse limit switches are enabled and configured
    // normally open
    m_winch.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_winch.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_winch.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

  }
  
  public void stop() {
    m_winch.set(ControlMode.PercentOutput, 0);
  }

  public void raiseRobot() {
    //if (endGame)
      m_winch.set(ControlMode.PercentOutput, WinchConstants.kRaiseRobotSpeed);
  } 

  public void lowerRobot() {
    //if (endGame)
      m_winch.set(ControlMode.PercentOutput, WinchConstants.kLowerRobotSpeed);
  }

  /**
   * For debug or possibly attach to joystick.
   * @param speed - the percent energy to send to the winch motor
   */
  public void runMotor(double speed) {
    //if (endGame) {
      double clampedSpeed = MathUtil.clamp(speed,WinchConstants.kMaxLowerRobotSpeed, WinchConstants.kMaxRaiseRobotSpeed);
      m_winch.set(ControlMode.PercentOutput, clampedSpeed);
    //}
  }

  /**
   * enable use of the winch during endGame to climb the chain
   * @param newEndGame the desired setting for endGame
   * @return original endGame value.
   */
  /*
  public boolean setEndGame(boolean newEndGame) {
    boolean result = endGame;
    endGame = newEndGame;
    return result;
  }
  */
  /**
   * get the current endGame setting.
   * @return the current endGame setting. True indicates winch operation is enabled.
   */
  /*public boolean getEndGame() {
    return endGame;
  }
  */

  @Override
  public void periodic() {
  }
}
