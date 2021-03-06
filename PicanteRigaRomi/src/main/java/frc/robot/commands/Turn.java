// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystem.DriveTrainSubsystem;

public class Turn extends CommandBase {
  DriveTrainSubsystem ds;
  double sp;
  /** Creates a new DriveStraight. */
  public Turn(RobotContainer robotContainer, double setpoint) {
    ds = robotContainer.driveTrainSubsystem;
    sp = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(robotContainer.driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ds.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ds.turnPID(sp, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ds.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ds.atsetPoint();
  }
}
