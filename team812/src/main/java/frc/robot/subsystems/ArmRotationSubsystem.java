// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.PidConstants;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.math.MathUtil;

public class ArmRotationSubsystem extends SubsystemBase {
  public final WPI_TalonSRX m_arm = new WPI_TalonSRX(CANConstants.kArmMotor);
  static Integer getPosition_timesCalled = 0;
  private static double targetPosition = 0;
  private boolean debug = false;
  private static double rotateTimesCalled=0;
  private static boolean m_rotateStopped = true;
  private static boolean m_capturedLimitPosition = false;
  

  /** Creates a new ArmSubsystem. */
  public ArmRotationSubsystem() {
    m_arm.configFactoryDefault();
    m_arm.setNeutralMode(NeutralMode.Brake);

    // This is a CLOSED loop system. Do not uncomment or enable
    // OpenloopRamp for the PID controlled arm.
    // m_arm.configOpenloopRamp(PidConstants.xxx_kArm_rampRate_xxx);

    // Configure the feedback sensor with the type (QuadEncoder),
    // the PID identifier within the Talon (pid 0) and the timeout (50ms)
    m_arm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 50);

    // Invert motor (setInverted) so that the Talon LEDs are green when driving
    // forward (up)
    // Phase sensor should have a positive increment as the Talon drives the arm up
    m_arm.setInverted(false);
    m_arm.setSensorPhase(true); // Attempts to make it positive

    // Set status frame period to 10ms with a timeout of 10ms
    // 10 sets timeouts for Motion Magic
    // 13 sets timeouts for PID 0
    m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);
    m_arm.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);

    // Configure low and high output levels to help remove any
    // stalling that might occur where stalling means that power is being
    // applied, but the motor isn't moving due to friction or inertia. This
    // can help the motor not burn itself out.
    m_arm.configNominalOutputForward(0, 10);
    m_arm.configNominalOutputReverse(0, 10);
    m_arm.configPeakOutputForward(ArmConstants.kArmPeakOutputForward, 10);
    m_arm.configPeakOutputReverse(ArmConstants.kArmPeakOutputReverse, 10);

    // Configure the Motion Magic parameters for PID 0 within the Talon
    // The values for P, I, D, and F will need to be determined emperically
    m_arm.selectProfileSlot(0, 0);
    m_arm.config_kP(0, PidConstants.kArm_kP, 10);
    m_arm.config_kI(0, PidConstants.kArm_kI, 10);
    m_arm.config_kD(0, PidConstants.kArm_kD, 10);
    m_arm.config_kF(0, PidConstants.kArm_kF, 10);
    m_arm.config_IntegralZone(0, PidConstants.kArm_IntegralZone, 10);

    // The next to configuration settings are for MotionMagic and are not used by the ControlMode.Position
    // Velocity in sensor units per 100ms
    m_arm.configMotionCruiseVelocity(150.0, 10);
    // Acceleration in sensor units per 100ms per second
    m_arm.configMotionAcceleration(150.0, 10);

    // Make sure the forward and reverse limit switches are enabled and configured
    // normally open
    m_arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
    m_arm.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

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
    if (debug) SmartDashboard.putNumber("rotate js", position);
    if (Math.abs(position) < 0.1) {  // Also move to constants.java
      if (!m_rotateStopped) {
        setPosition(getPosition());
        m_rotateStopped = true;
      }
    } else {
      m_rotateStopped = false;
      double newPosition = currentTarget + position * incrementSize;
      newPosition = MathUtil.clamp(newPosition, ArmConstants.kArmMinPosition, ArmConstants.kArmMaxPosition);
      if (debug) SmartDashboard.putNumber("rotate pos", newPosition);
      setPosition(newPosition);
      rotateTimesCalled++;
    }
  };

  public void rotateUp50() {
    setPosition(targetPosition+50.0);
  }

  public void rotateDown50() {
    setPosition(targetPosition-50.0);
  }

  public void stop() {
    m_arm.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor(double speed) {

    double clampedSpeed = MathUtil.clamp(speed, ArmConstants.kArmPeakOutputReverse, ArmConstants.kArmPeakOutputForward);
    m_arm.set(ControlMode.PercentOutput, clampedSpeed);
  }

  // Set the arm target position after checking that it is safe to do so.
  public double setPosition(double position) {
    // position will be zero in tucked position
    if (isHome() && position >= ArmConstants.kArmMinPosition && position <= ArmConstants.kArmMaxPosition) {
      if (debug) SmartDashboard.putNumber("ArmSubPos", position);
      m_arm.set(ControlMode.Position, position);
      targetPosition = position;
    }
    return getPosition();
  }

  // Only to be used when homing the robot
  // This sets the goal encoder value without checking to see if it is reasonable.
  public double setHomePosition(double position) {
    m_arm.set(ControlMode.Position, position);
    m_capturedLimitPosition = false;
    return getPosition();
  }

  public double getPosition() {
    double position = m_arm.getSelectedSensorPosition(0);
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
    m_arm.setSelectedSensorPosition(position, 0, 10);
  }

  // This function sets the arm encoder to the expected arm starting position
  public void setSensorReference() {
    double l_position = ArmConstants.kArmStartingPosition;
    m_arm.setSelectedSensorPosition(l_position, 0, 10);
    setHomePosition(l_position);
    setHome(); 
  }

  // Returns true if the arm is fully lowered.
  public boolean isFwdLimitSwitchClosed() {
    return (m_arm.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  // Returns true if the arm is fully raised.
  public boolean isRevLimitSwitchClosed() {
    return (m_arm.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  public void setHome() {
    m_capturedLimitPosition = true;
    System.out.println("setHome m_capturedLimitPosition: " + m_capturedLimitPosition);
     if (isFwdLimitSwitchClosed()) {
//        SmartDashboard.putNumber("ARM pos rev limit", getPosition());
        m_capturedLimitPosition = true;
        setSensorPosition(Constants.ArmConstants.kArmMaxPosition);
//      }
    }
  }

  public void unsetHome() {
    m_capturedLimitPosition = false;
  }

  public void unsetHome(String msg) {
    m_capturedLimitPosition = false;
  }

  public boolean isHome() {
    return m_capturedLimitPosition;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!m_capturedLimitPosition) {
      if (isFwdLimitSwitchClosed()) {
        setHome(); // Added 2024-03-13
      }
    }
    if (debug) {
      SmartDashboard.putNumber("Arm pos:", getPosition());
      SmartDashboard.putNumber("Arm target", targetPosition);
      SmartDashboard.putBoolean("Arm Homed?", isHome());
      SmartDashboard.putBoolean("ARM fwdsw closed", isFwdLimitSwitchClosed());
      SmartDashboard.putBoolean("ARM revsw closed",isRevLimitSwitchClosed());
    }

    if (debug) {
      SmartDashboard.putNumber("ARM rotate calls", rotateTimesCalled);
      //SmartDashboard.putNumber("ARM Output%", m_arm.getMotorOutputPercent());
      //SmartDashboard.putNumber("ARM Voltage", m_arm.getMotorOutputVoltage());
      SmartDashboard.putNumber("ARM Output%", m_arm.getMotorOutputPercent());
      SmartDashboard.putNumber("ARM Voltage", m_arm.getMotorOutputVoltage());
      SmartDashboard.putNumber("ARM error", m_arm.getClosedLoopError(0));
      ControlMode controlMode = m_arm.getControlMode();
      SmartDashboard.putString("ARM ctlrmode", controlMode.toString());
    }
    // If the forward limit switch is closed, we are fully rotated to the Algae intake position.
    // If we have not done so already, reset the encoder coordinates to the fully rotated value.
   
  }
}
