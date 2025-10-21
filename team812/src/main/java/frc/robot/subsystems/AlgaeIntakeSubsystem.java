// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeIntakeConstants;
import frc.utils.PreussMotor;
import frc.utils.PreussMotorConfig;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  public PreussMotor m_AlgaeIntake;

  /** Creates a new AlgaeIntake Subsystem. */
  public AlgaeIntakeSubsystem(PreussMotorConfig config) {
    m_AlgaeIntake = new PreussMotor(config);
  }

  public void stop() {
    m_AlgaeIntake.stop();
  }

  // TODO: Are there limit switches for the Algae handler?
  public boolean isTopLimitSwitchClosed() {
    return (m_AlgaeIntake.isFwdLimitSwitchClosed() == 1 ? true : false);
  }

  public boolean isBottomLimitSwitchClosed() {
    return (m_AlgaeIntake.isRevLimitSwitchClosed() == 1 ? true : false);
  }

  public void pickUpAlgae() {
    m_AlgaeIntake.runMotor(AlgaeIntakeConstants.kPickUpAlgaeSpeed);
  } 

  public void expelAlgae() {
        m_AlgaeIntake.runMotor(AlgaeIntakeConstants.kExpelAlgaeSpeed);
  }

  public void  runMotor(double speed) {
    double clampedSpeed = MathUtil.clamp(speed,AlgaeIntakeConstants.kExpelAlgaeSpeed,AlgaeIntakeConstants.kPickUpAlgaeSpeed);
    m_AlgaeIntake.runMotor(clampedSpeed);
  } 

  @Override
  public void periodic() {
  }
  
}