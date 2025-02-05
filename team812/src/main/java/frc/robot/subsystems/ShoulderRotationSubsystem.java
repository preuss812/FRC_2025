// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShoulderConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.utils.PreussMotor;

public class ShoulderRotationSubsystem extends SubsystemBase {
  public final PreussMotor m_shoulder = new PreussMotor(Constants.shoulderMotor);
  private static double targetPosition = 0;
  private static boolean m_rotateStopped = true;
  private static boolean m_capturedLimitPosition = false;
  private static AnalogEncoder m_encoder = new AnalogEncoder(ShoulderConstants.kShoulderEncoderInputChannel);
  // In case we need AnalogPotentiometer instead of AnalogEncoder:  TalonSRX Software Reference Manual 7.5.2
  //private static AnalogPotentiometer m_potentiometer = new AnalogPotentiometer(ShoulderConstants.kShoulderEncoderInputChannel);

  /** Creates a new ArmSubsystem. */
  public ShoulderRotationSubsystem() {
  }

  private final int incrementSize = 50; // Move to Constants.java?  // 5*50 = 250 per second = 10 degrees per second when joystick maxed out. TODO tune this

  // This function is used as the default command to run for arm control.
  // The input is presumed to be an analog input that ranges from -1 to +1.
  public void rotate(double position) {
    //double absolutePosition = getPosition(); // Should get the goal, not the position. // Dont need it.
    double currentTarget = targetPosition;
    // if the joystick is nearly centered, ignore it
    // This has the effect of stopping the arm rotation if the joystick is not being used to control the arm.
    // Be aware that if another command ends before it gets the arm to the desired position,
    // this function will stop the arm motiion and it will not continue rotating to the other commands target.
    if (Math.abs(position) < 0.1) {  // Also move to constants.java
      if (!m_rotateStopped) {
        setTargetPosition(getPosition());
        m_rotateStopped = true;
      }
    } else {
      m_rotateStopped = false;
      double newPosition = currentTarget + position * incrementSize;
      newPosition = MathUtil.clamp(newPosition, ShoulderConstants.kShoulderMinPosition, ShoulderConstants.kShoulderMaxPosition);
      setTargetPosition(newPosition);
    }
    m_encoder.setVoltagePercentageRange(0,3.3); // TODO make constants and find out if 3.3 or 5.0 or some internal range.
    m_shoulder.configSelectedFeedbackSensor(FeedbackDevice.Analog, ShoulderConstants.kPidIdx, ShoulderConstants.kTimeoutMs);
    
  };

  public void rotateUp50() {
    setTargetPosition(targetPosition+50.0);
  }

  public void rotateDown50() {
    setTargetPosition(targetPosition-50.0);
  }

  public void stop() {
    m_shoulder.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor(double speed) {

    double clampedSpeed = MathUtil.clamp(speed, ShoulderConstants.kShoulderPeakOutputReverse, ShoulderConstants.kShoulderPeakOutputForward);
    m_shoulder.set(ControlMode.PercentOutput, clampedSpeed);
  }

  // Set the arm target position after checking that it is safe to do so.
  public double setTargetPosition(double position) {
    // position will be zero in tucked position
    if (isHomed() && position >= ShoulderConstants.kShoulderMinPosition && position <= ShoulderConstants.kShoulderMaxPosition) {
      m_shoulder.set(ControlMode.Position, position);
      targetPosition = position;
    }
    return getPosition();
  }

  public double getPosition() {
    double position = m_shoulder.getSelectedSensorPosition(0);
    return position;
  }

  public double getTargetPosition() {
      return targetPosition;
  }

  public double getPositionError() {
    return getPosition() - getTargetPosition();
  }

  // Sets the target encoder value.  The PID in the TalonSRX will drive the arm to this position.
  public void setSensorPosition(double position) {
    m_shoulder.setSelectedSensorPosition(position, 0, 10);
  }

  // Returns true if the arm is fully lowered.
  public boolean isFwdLimitSwitchClosed() {
    return (m_shoulder.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the arm is fully raised.
  public boolean isRevLimitSwitchClosed() {
    return (m_shoulder.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the shoulder is at the home position
  public boolean isAtHome() {
    return isFwdLimitSwitchClosed();
  }

  public void setHomed() {
    m_capturedLimitPosition = true;
    if (isAtHome()) {
      // Remeber that we are homed and set the encoder coordinates to the home position and try to hold it there.
      m_capturedLimitPosition = true;
      m_shoulder.setSelectedSensorPosition(ShoulderConstants.kShoulderHomePosition, Constants.shoulderMotor.pidIdx, Constants.shoulderMotor.timeout);
      setTargetPosition(Constants.ShoulderConstants.kShoulderHomePosition);
    }
  }

  public void unsetHomed() {
    m_capturedLimitPosition = false;
  }

  public boolean isHomed() {
    return m_capturedLimitPosition;
  }

  // This method is for testing before the robot is fully built.
  // Do NOT use after the robot is built and the limit switches are availble
  public void testSetHomed() {
    m_capturedLimitPosition = true;
    m_shoulder.setSelectedSensorPosition(ShoulderConstants.kShoulderHomePosition, Constants.shoulderMotor.pidIdx, Constants.shoulderMotor.timeout);
    setTargetPosition(Constants.ShoulderConstants.kShoulderHomePosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isHomed()) {
      if (isAtHome()) {
        setHomed();
      }
    }
    SmartDashboard.putNumber("Shoulder Pos",   getPosition());
    SmartDashboard.putNumber("Shoulder target", targetPosition);
    SmartDashboard.putBoolean("Shoulder Homed", isHomed());
    SmartDashboard.putBoolean("Shoulder fwdsw", isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("Shoulder revsw", isRevLimitSwitchClosed());
    SmartDashboard.putNumber("Shoulder encoder:", m_encoder.get());
    int analogPos = m_shoulder.getSensorCollection().getAnalogIn();
    SmartDashboard.putNumber("Shoulder analogPos:", analogPos);
    // function apparently deprecated and may not work for analog
    //FeedbackDeviceStatus encoderStatus = m_shoulder.isSensorPresent(FeedbackDevice.Analog);

  }
}
