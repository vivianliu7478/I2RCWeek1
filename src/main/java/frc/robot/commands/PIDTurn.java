// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {
  DriveTrain dt;
  double setpointAngle;
  PIDController pid = new PIDController (0.3/90, 0, 0);
  int motorSign;
  
  

  /** Creates a new PIDTurn. */
  public PIDTurn(DriveTrain dt, double setpointAngle) {
    this.dt = dt;
    this.setpointAngle = setpointAngle;
    addRequirements(dt);
    if (setpointAngle >= 1){ //It will counter-clockwise turn if the motor is at 1
      motorSign = 1;
    }
    else{
      motorSign = -1;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.resetNavx();
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
