// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnalogIOConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Utilities;
import frc.utils.PreussMotor;

public class ElbowRotationSubsystem extends SubsystemBase {
  public final PreussMotor m_elbowLeft = new PreussMotor(Constants.elbowMotor1);
  //public final PreussMotor m_elbowRight = new PreussMotor(Constants.elbowMotor2);
  private static AnalogInput m_AnalogInput = new AnalogInput(AnalogIOConstants.kElbowEncoder);
  //private static AnalogEncoder m_encoder = new AnalogEncoder(m_AnalogInput, 3.3, 0.0);
  private static PIDController m_pidController = new PIDController(PidConstants.kElbow_kP, PidConstants.kElbow_kI, PidConstants.kElbow_kD);
  private static double targetPosition;  
  private static double currentPosition;
  private static double analogPosition; 
  private static boolean m_capturedLimitPosition = true; // This year absolute encoder so we do not need to home the arm.
  private boolean m_rotateStopped = true;
  private static boolean debug = true; // TODO: Set to false once the elbow is debugged.

  // Note: This year's encoder is a Lamprey II which outputs from 0 to 360 degrees (Not sure about wrap around or minus)

  /** Creates a new ArmSubsystem. */
  public ElbowRotationSubsystem() {
    // Have the right motor do what the left motor does so they work together
    //m_elbowRight.follow(m_elbowLeft);
    stop(); // Make sure the motor is not moving
    readCurrentPosition();
    currentPosition = getCurrentPosition(); // Get the arm's current position and make that the target position.
    targetPosition = currentPosition;  // initially hold the starting arm position.
  }

  private final double incrementSize = 0.5; // 0.5*50 periods per second = 25 degrees per second = when joystick maxed out. TODO tune this

  // This function is used as the default command to run for arm control.
  // The input is presumed to be an analog input that ranges from -1 to +1.
  public void rotate(double throttle) {
    //double absolutePosition = getPosition(); // Should get the goal, not the position. // Dont need it.
    // if the joystick is nearly centered, ignore it
    // This has the effect of stopping the arm rotation if the joystick is not being used to control the arm.
    // Be aware that if another command ends before it gets the arm to the desired position,
    // this function will stop the arm motiion and it will not continue rotating to the other commands target.
    if (Math.abs(throttle) < ElbowConstants.kElbowDeadband) {  
      if (!m_rotateStopped) {
        setTargetPosition(getCurrentPosition());
        m_rotateStopped = true;
      }
    } else {
      double newPosition = targetPosition + throttle * incrementSize;
      newPosition = MathUtil.clamp(newPosition, ElbowConstants.kElbowMinPosition, ElbowConstants.kElbowMaxPosition);
      setTargetPosition(newPosition);
    }
  };

  // Probably deadwood.
  @Deprecated
  public void rotateUp50() {
    setTargetPosition(targetPosition+50.0);
  }

  // Probably deadwood.
  @Deprecated
  public void rotateDown50() {
    setTargetPosition(targetPosition-50.0);
  }

  public void stop() {
    m_elbowLeft.set(ControlMode.PercentOutput, 0.0);
    m_rotateStopped = true;
  }

  public void runMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed, ElbowConstants.kElbowPeakOutputReverse, ElbowConstants.kElbowPeakOutputForward);
    m_elbowLeft.set(ControlMode.PercentOutput, clampedSpeed);
  }

  // Set the arm target position after checking that it is safe to do so.
  public double setTargetPosition(double position) {
    // position will be zero in tucked position
    if (isHomed() && position >= ElbowConstants.kElbowMinPosition && position <= ElbowConstants.kElbowMaxPosition) {
      //m_elbowLeft.set(ControlMode.Position, position);
      targetPosition = position;
    }
    return getCurrentPosition();
  }

  public double getCurrentPosition() {
    //double position = m_elbowLeft.getSelectedSensorPosition(Constants.elbowMotor1.pidIdx);
    return currentPosition;
  }

  public double getTargetPosition() {
      return targetPosition;
  }

  public double getPositionError() {
    return getCurrentPosition() - getTargetPosition();
  }

   // Sets the target encoder value.  The PID in the TalonSRX will drive the arm to this position.
   @Deprecated
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

  // Not needed due to absolute encoder
  @Deprecated
  public void setHomed() {
      if (isAtHome()) {
        // Remeber that we are homed and set the encoder coordinates to the home position and try to hold it there.
        m_capturedLimitPosition = true;
        m_elbowLeft.setSelectedSensorPosition(ElbowConstants.kElbowHomePosition, Constants.elbowMotor1.pidIdx, Constants.elbowMotor1.timeout);
        setTargetPosition(Constants.ElbowConstants.kElbowHomePosition);
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
    m_elbowLeft.setSelectedSensorPosition(ElbowConstants.kElbowHomePosition, Constants.elbowMotor1.pidIdx, Constants.elbowMotor1.timeout);
    setTargetPosition(Constants.ElbowConstants.kElbowHomePosition);
  }

  public void readCurrentPosition() {
    analogPosition = m_AnalogInput.getAverageVoltage();
    currentPosition=Utilities.scaleDouble(
       analogPosition
      , ElbowConstants.kElbowMinPosition
      , ElbowConstants.kElbowMaxPosition
      , ElbowConstants.kElbowMinEncoderVoltage
      , ElbowConstants.kElbowMaxEncoderVoltage
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
    double percentOutput=MathUtil.clamp(m_pidController.calculate(error), ElbowConstants.kElbowPeakOutputReverse, ElbowConstants.kElbowPeakOutputForward);
    m_elbowLeft.set(ControlMode.PercentOutput, percentOutput);
    if (debug) {
      SmartDashboard.putNumber("Elbow Pos",    getCurrentPosition());
      SmartDashboard.putNumber("Elbow target", getTargetPosition());
      SmartDashboard.putNumber("Elbow error",  getPositionError());
      SmartDashboard.putBoolean("Elbow Homed", isHomed());
      SmartDashboard.putBoolean("Elbow fwdsw", isFwdLimitSwitchClosed());
      SmartDashboard.putBoolean("Elbow revsw", isRevLimitSwitchClosed());
      SmartDashboard.putNumber("Elbow output", percentOutput);
      //SmartDashboard.putNumber("Elbow input",  m_AnalogInput.getAverageVoltage());
      SmartDashboard.putNumber("Elbow encoder", analogPosition);
    }
  }
}
