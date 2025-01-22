// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElbowRotationSubsystem;
import frc.robot.subsystems.ShoulderRotationSubsystem;

/**
 * This command raises the arm, shoots the Algae and lowers the arm sequentially.
 */
public class ScoreAlgaeInProcessor extends SequentialCommandGroup {
  /** Creates a new ScoreAlgaeInProcessor. */
  public ScoreAlgaeInProcessor(
    AlgaeIntakeSubsystem algaeIntakeSubsystem,
    ElbowRotationSubsystem elbowRotationSubsystem,
    ShoulderRotationSubsystem shoulderRotationSubsystem
    ) {
    
    addCommands(
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmUP")),
      new ElbowRotationCommand(elbowRotationSubsystem, ElbowConstants.kElbowScoringPosition).withTimeout(ElbowConstants.kElbowRaiseTimeout),
      //new InstantCommand(()->Utilities.resetPoseAtProcessor()),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "Shoot")),
      new ShoulderRotationCommand(shoulderRotationSubsystem, ShoulderConstants.kShoulderProcessorPosition).withTimeout(ShoulderConstants.kShoulderTimeout),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ArmDown")),
      new ElbowRotationCommand(elbowRotationSubsystem, ElbowConstants.kElbowMaxPosition).withTimeout(ElbowConstants.kElbowLowerTimeout),
      new InstantCommand(() -> SmartDashboard.putString("ActiveCommand", "ScoreAlgaeDone"))
    );

  }
}
