// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Utilities;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.utils.PreussMotor;

public class ShoulderRotationSubsystem extends SubsystemBase {
  public final PreussMotor m_shoulder = new PreussMotor(Constants.shoulderMotor);
  // Note: None of these variable need to be "static"
  private static double analogPosition;
  private static double targetPosition;
  private static double currentPosition;
  private static boolean m_rotateStopped = true;
  private static boolean m_capturedLimitPosition = true; // Due to absolute encoder, we are always homed.
  private static AnalogInput m_analogInput = new AnalogInput(AnalogIOConstants.kShoulderEncoder);
  //private static AnalogEncoder m_encoder = new AnalogEncoder(m_analogInput);
  private static PIDController m_pidController = new PIDController(PidConstants.kShoulder_kP, PidConstants.kShoulder_kI, PidConstants.kShoulder_kD);
  private static boolean debug = true; // TODO: Set to false once the shoulder is debugged.

  /** Creates a new ArmSubsystem. */
  public ShoulderRotationSubsystem() { 
    stop(); // Make sure the motor is not moving
    readCurrentPosition();
    targetPosition = currentPosition;  // initially hold the starting arm position.
  }

  private final double incrementSize = 0.5; // 0.5*50cycles/sec = 25 degrees per second when joystick maxed out. TODO tune this

  // This function is used as the default command to run for arm control.
  // The input is presumed to be an analog input that ranges from -1 to +1.
  public void rotate(double throttle) {
    //double absolutePosition = getPosition(); // Should get the goal, not the position. // Dont need it.
    // if the joystick is nearly centered, ignore it
    // This has the effect of stopping the arm rotation if the joystick is not being used to control the arm.
    // Be aware that if another command ends before it gets the arm to the desired position,
    // this function will stop the arm motiion and it will not continue rotating to the other commands target.
    if (Math.abs(throttle) < 0.1) {  // Also move to constants.java
      if (!m_rotateStopped) {
        setTargetPosition(getCurrentPosition());
        m_rotateStopped = true;
      }
    } else {
      m_rotateStopped = false;
      double newPosition = targetPosition + throttle * incrementSize;
      newPosition = MathUtil.clamp(newPosition, ShoulderConstants.kShoulderMinPosition, ShoulderConstants.kShoulderMaxPosition);
      setTargetPosition(newPosition);
    }
    m_shoulder.configSelectedFeedbackSensor(FeedbackDevice.Analog, ShoulderConstants.kPidIdx, ShoulderConstants.kTimeoutMs);
    
  };

  @Deprecated
  public void rotateUp50() {
    setTargetPosition(targetPosition+50.0);
  }

  @Deprecated
  public void rotateDown50() {
    setTargetPosition(targetPosition-50.0);
  }

  public void stop() {
    m_shoulder.set(ControlMode.PercentOutput, 0);
  }

  /**
   * runMotor - run the motor at the specified output percentage.
   * @param speed - the percent motor output to use ranging from -1.0 to 1.0.
   */
  public void runMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, ShoulderConstants.kShoulderPeakOutputReverse, ShoulderConstants.kShoulderPeakOutputForward);
    m_shoulder.set(ControlMode.PercentOutput, clampedSpeed);
  }

  /**
   * setTargetPosition - Set the arm target position after checking that it is safe to do so.
   * @param - the target angle for the arm in degrees
   * @return - the current angle of the arm.
   */
  public double setTargetPosition(double position) {
    // position will be zero in tucked position
    if (isHomed() && position >= ShoulderConstants.kShoulderMinPosition && position <= ShoulderConstants.kShoulderMaxPosition) {
      m_shoulder.set(ControlMode.Position, position);
      targetPosition = position;
    }
    return getCurrentPosition();
  }

  /**
   * getCurrentPosition - get the current angle of rotation for the shoulder joint.
   * @return - the current angle of rotation in degrees
   */
  public double getCurrentPosition() {
    return currentPosition; // Relying on periodic to keep currentPosition fresh.
  }

  /**
   * getTargetPosition - get the target angle of rotation for the shoulder joint
   * @return - the target angle of rotation of the shoulder join in degrees.
   */
  public double getTargetPosition() {
      return targetPosition;
  }

  /**
   * getPositionError - get the difference between the current and target angles.
   * @return - the difference between the current and target angles in degrees.
   */
  public double getPositionError() {
    return getCurrentPosition() - getTargetPosition();
  }

  // Sets the target encoder value.  The PID in the TalonSRX will drive the arm to this position.
  @Deprecated // This is used when the encoder is wired to the talon.
  public void setSensorPosition(double position) {
    m_shoulder.setSelectedSensorPosition(position, 0, 10);
  }

  // Returns true if the arm is fully lowered.
  // I'd prefer names to be upper and lower but have not made that change.
  public boolean isFwdLimitSwitchClosed() {
    return (m_shoulder.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the arm is fully raised.
  // I'd prefer names to be upper and lower but have not made that change.
  public boolean isRevLimitSwitchClosed() {
    return (m_shoulder.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the shoulder is at the home position
  public boolean isAtHome() {
    return isFwdLimitSwitchClosed();
  }

  @Deprecated
  public void setHomed() {
    m_capturedLimitPosition = true;
    if (isAtHome()) {
      // Remeber that we are homed and set the encoder coordinates to the home position and try to hold it there.
      m_capturedLimitPosition = true;
      m_shoulder.setSelectedSensorPosition(ShoulderConstants.kShoulderHomePosition, Constants.shoulderMotor.pidIdx, Constants.shoulderMotor.timeout);
      setTargetPosition(Constants.ShoulderConstants.kShoulderHomePosition);
    }
  }

  @Deprecated
  public void unsetHomed() {
    m_capturedLimitPosition = false;
  }

  public boolean isHomed() {
    return m_capturedLimitPosition;
  }

  // This method is for testing before the robot is fully built.
  // Do NOT use after the robot is built and the limit switches are availble
  @Deprecated
  public void testSetHomed() {
    m_capturedLimitPosition = true;
    m_shoulder.setSelectedSensorPosition(ShoulderConstants.kShoulderHomePosition, Constants.shoulderMotor.pidIdx, Constants.shoulderMotor.timeout);
    setTargetPosition(Constants.ShoulderConstants.kShoulderHomePosition);
  }

  public void readCurrentPosition() {
    analogPosition = m_analogInput.getAverageVoltage(); // Intentionally NOT using encoder. dph - 2025-03-03
    currentPosition=Utilities.scaleDouble(
      analogPosition
      , ShoulderConstants.kShoulderMinPosition
      , ShoulderConstants.kShoulderMaxPosition
      , ShoulderConstants.kShoulderMinEncoderVoltage
      , ShoulderConstants.kShoulderMaxEncoderVoltage
    ); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isHomed()) {
      if (isAtHome()) {
        setHomed();
      }
    }
    
    readCurrentPosition();
    double error=getPositionError(); 
    double percentOutput=MathUtil.clamp(m_pidController.calculate(error), ShoulderConstants.kShoulderPeakOutputReverse, ShoulderConstants.kShoulderPeakOutputForward);
    m_shoulder.set(ControlMode.PercentOutput, percentOutput);
    if (debug) {
      SmartDashboard.putNumber("Shoulder Pos",    currentPosition);
      SmartDashboard.putNumber("Shoulder input",  analogPosition);
      SmartDashboard.putNumber("Shoulder output", percentOutput);
      SmartDashboard.putNumber("Shoulder target", targetPosition);
      SmartDashboard.putBoolean("Shoulder Homed", isHomed());
      SmartDashboard.putBoolean("Shoulder fwdsw", isFwdLimitSwitchClosed());
      SmartDashboard.putBoolean("Shoulder revsw", isRevLimitSwitchClosed());
    }
  }
}
