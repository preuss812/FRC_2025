// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.AlgaeIntakeConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_AlgaeIntake = new WPI_TalonSRX(CANConstants.kAlgaeIntakeMotor);
  //public final WPI_TalonSRX m_AlgaeIntake2 = new WPI_TalonSRX(CANConstants.kAlgaeIntakeMotor2);

  
  /** Creates a new ArmSubsystem. */
  public AlgaeIntakeSubsystem() {
    m_AlgaeIntake.configFactoryDefault();
    m_AlgaeIntake.setNeutralMode(NeutralMode.Brake);

    // Configure the feedback sensor with the type (QuadEncoder),
    // the PID identifier within the Talon (pid 0) and the timeout (50ms)
    m_AlgaeIntake.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);  // TODO Remove this and possible modify for limit switch/sensor

    // Invert motor (setInverted) so that the Talon LEDs are green when driving
    // forward (up)
    // Phase sensor should have a positive increment as the Talon drives the arm up
    m_AlgaeIntake.setInverted(true);
    m_AlgaeIntake.setSensorPhase(true); // Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_AlgaeIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_AlgaeIntake.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_AlgaeIntake.configNominalOutputForward(0, 10);
    m_AlgaeIntake.configNominalOutputReverse(0, 10);
    m_AlgaeIntake.configPeakOutputForward(0.6, 10);
    m_AlgaeIntake.configPeakOutputReverse(-0.6, 10);

    // Configure the Motion Magic parameters for PID 0 within the Talon
    // The values for P, I, D, and F will need to be determined emperically
    m_AlgaeIntake.selectProfileSlot(0, 0);
    m_AlgaeIntake.config_kP(0, PidConstants.kAlgaeIntake_kP, 10);
    m_AlgaeIntake.config_kI(0, PidConstants.kAlgaeIntake_kI, 10);
    m_AlgaeIntake.config_kD(0, PidConstants.kAlgaeIntake_kD, 10);
    m_AlgaeIntake.config_kF(0, PidConstants.kAlgaeIntake_kF, 10);

    // Velocity in sensor units per 100ms
    m_AlgaeIntake.configMotionCruiseVelocity(150.0, 10);
    // Acceleration in sensor units per 100ms per second
    m_AlgaeIntake.configMotionAcceleration(150.0, 10);

    // Make sure the forward and reverse limit switches are enabled and configured
    // normally open
    m_AlgaeIntake.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_AlgaeIntake.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_AlgaeIntake.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

  }

  public void stop() {
    m_AlgaeIntake.set(ControlMode.PercentOutput, 0);
  }

  public boolean isTopLimitSwitchClosed() {
    return (m_AlgaeIntake.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  public boolean isBottomLimitSwitchClosed() {
    return (m_AlgaeIntake.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  public void pickUpAlgae() {
    m_AlgaeIntake.set(ControlMode.PercentOutput, AlgaeIntakeConstants.kPickUpAlgaeSpeed);
  } 

  public void expelAlgae() {
        m_AlgaeIntake.set(ControlMode.PercentOutput, AlgaeIntakeConstants.kExpelAlgaeSpeed);
  }

  public void runMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed,AlgaeIntakeConstants.kExpelAlgaeSpeed,AlgaeIntakeConstants.kPickUpAlgaeSpeed);
    m_AlgaeIntake.set(ControlMode.PercentOutput, clampedSpeed);
  } 

  @Override
  public void periodic() {
  }
  
}
