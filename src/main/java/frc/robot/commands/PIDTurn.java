// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.USBOrder;
import frc.robot.subsystems.DriveTrain;

public class PIDTurn extends CommandBase {
  DriveTrain dt;
  double setpointAngle;
  PIDController pid = new PIDController(USBOrder.kP, 0, 0);
  int motorSign;
  
  

  /** Creates a new PIDTurn. */
  public PIDTurn(DriveTrain dt, double setpointAngle) {
    this.dt = dt;
    this.setpointAngle = setpointAngle;
    addRequirements(dt);
    if (setpointAngle >= 0){ //It will counter-clockwise turn if the motor is at 1
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
    dt.tankDrive(0, 0);
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(dt.getAngle(), setpointAngle);
    dt.tankDrive(motorSign * output, motorSign * output);

  }

  // Called once the command ends or is interrupted.
  // the robot doesn't go forward because it doesnt reach the setpoint so we have to make the set tolerance bigger
  // the set tolerance is the way it is able to control the amount of power that the robot can have then.
  @Override
  public void end(boolean interrupted) {
    pid.setTolerance(5.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
