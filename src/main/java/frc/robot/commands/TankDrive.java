// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TankDrive extends CommandBase {
  public DriveTrain dt;
  public Joystick joy;

  /** Creates a new TankDrive. */
// The constructor 
  public TankDrive(DriveTrain dt, Joystick j) {
    this.dt = dt;
    this.joy = j;
  // Creating a DriveTrain and a Joystick

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }
// Initializes both the talons to be zero
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dt.tankDrive(0.0, 0.0);
    dt.resetEncoders();
  }
// Gets raw data from left JoyStick and right JoyStick
// Setting speed to the motors relative to the raw data from the JoySticks
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftPowerRaw = joy.getRawAxis(1);

    double rightPowerRaw = joy.getRawAxis(5);

    dt.tankDrive(leftPowerRaw*-0.7, rightPowerRaw*-0.7);
  }
// Sets the speed to zero
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.tankDrive(0.0, 0.0);
    dt.resetEncoders();
  }
// Returning is finished when the command ends
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
