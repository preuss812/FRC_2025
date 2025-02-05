// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElbowConstants;
import frc.utils.PreussMotor;

public class ElbowRotationSubsystem extends SubsystemBase {
  public final PreussMotor m_elbowLeft = new PreussMotor(Constants.elbowMotor1);
  public final PreussMotor m_elbowRight = new PreussMotor(Constants.elbowMotor2);

  private static double targetPosition = 0;  
  private static boolean m_capturedLimitPosition = false;
  private boolean m_rotateStopped = true;
  

  /** Creates a new ArmSubsystem. */
  public ElbowRotationSubsystem() {
    // Have the right motor do what the left motor does so they work together
    m_elbowRight.follow(m_elbowLeft);
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
      double newPosition = currentTarget + position * incrementSize;
      newPosition = MathUtil.clamp(newPosition, ElbowConstants.kElbowMinPosition, ElbowConstants.kElbowMaxPosition);
      setTargetPosition(newPosition);
    }
  };

  public void rotateUp50() {
    setTargetPosition(targetPosition+50.0);
  }

  public void rotateDown50() {
    setTargetPosition(targetPosition-50.0);
  }

  public void stop() {
    m_elbowLeft.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, ElbowConstants.kElbowPeakOutputReverse, ElbowConstants.kElbowPeakOutputForward);
    m_elbowLeft.set(ControlMode.PercentOutput, clampedSpeed);
  }

  // Set the arm target position after checking that it is safe to do so.
  public double setTargetPosition(double position) {
    // position will be zero in tucked position
    if (isHomed() && position >= ElbowConstants.kElbowMinPosition && position <= ElbowConstants.kElbowMaxPosition) {
      m_elbowLeft.set(ControlMode.Position, position);
      targetPosition = position;
    }
    return getPosition();
  }

  public double getPosition() {
    double position = m_elbowLeft.getSelectedSensorPosition(Constants.elbowMotor1.pidIdx);
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
    m_elbowLeft.setSelectedSensorPosition(position, 0, 10);
  }

  // Returns true if the arm is fully lowered.
  public boolean isFwdLimitSwitchClosed() {
    return (m_elbowLeft.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the arm is fully raised.
  public boolean isRevLimitSwitchClosed() {
    return (m_elbowLeft.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  // Return elbow at home position
  public boolean isAtHome() {
    return isFwdLimitSwitchClosed(); // TODO should this be forward or reverse?
  }

  public void setHomed() {
      if (isAtHome()) {
        // Remeber that we are homed and set the encoder coordinates to the home position and try to hold it there.
        m_capturedLimitPosition = true;
        m_elbowLeft.setSelectedSensorPosition(ElbowConstants.kElbowHomePosition, Constants.elbowMotor1.pidIdx, Constants.elbowMotor1.timeout);
        setTargetPosition(Constants.ElbowConstants.kElbowHomePosition);
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
    m_elbowLeft.setSelectedSensorPosition(ElbowConstants.kElbowHomePosition, Constants.elbowMotor1.pidIdx, Constants.elbowMotor1.timeout);
    setTargetPosition(Constants.ElbowConstants.kElbowHomePosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isHomed()) {
      if (isAtHome()) {
        setHomed();
      }
    }
    
    SmartDashboard.putNumber("Elbow Pos",   getPosition());
    SmartDashboard.putNumber("Elbow target", targetPosition);
    SmartDashboard.putBoolean("Elbow Homed", isHomed());
    SmartDashboard.putBoolean("Elbow fwdsw", isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("Elbow revsw", isRevLimitSwitchClosed());
  
  }
}
