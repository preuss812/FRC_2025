// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElbowRotationSubsystem;
import frc.robot.subsystems.ShoulderRotationSubsystem;

/**
 * This class accepts goal shoulder and elbow angles and manipulates the arm to that configuration 
 */
public class CompoundArmMovementCommand extends ParallelCommandGroup {
  private final ElbowRotationSubsystem m_elbowRotationSubsystem;
  private final ShoulderRotationSubsystem m_shoulderRotationSubsystem;

  /** Creates a new CompoundArmMovementCommand. */
  public CompoundArmMovementCommand(
    ElbowRotationSubsystem elbowRotationSubsystem
    , ShoulderRotationSubsystem shoulderRotationSubsystem
    , double elbowPosition, double shoulderPosition
    , boolean simulation
    ) {
    
    this.m_elbowRotationSubsystem = elbowRotationSubsystem;
    this.m_shoulderRotationSubsystem = shoulderRotationSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElbowRotationCommand(m_elbowRotationSubsystem, elbowPosition, simulation),
      new ShoulderRotationCommand(m_shoulderRotationSubsystem, shoulderPosition, simulation)
    );
  }
}
