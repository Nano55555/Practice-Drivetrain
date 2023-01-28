// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.driveSubsystem;

public class JoystickDrive extends CommandBase {
  /** Creates a new JoystickDrive. */
  public static final XboxController drivecontroller = RobotContainer.driverController;
  public static driveSubsystem driveSubsystem = new driveSubsystem();
  public JoystickDrive(driveSubsystem drivetrain) {
    driveSubsystem = drivetrain;
    addRequirements(driveSubsystem);

  } 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drive = drivecontroller.getLeftY();
    double rotate = drivecontroller.getRightX();
    if ((drive <0.25 && drive >0)|| (drive <0 && drive >-0.25)){
      drive =0;
    }
    else {
      drive= drive *0.5;
    }
    if ((rotate>0 &&rotate<0.25)|| (rotate<0 &&rotate >-0.25)){
      rotate =0;
    }
    else {
      rotate = rotate *0.25;
    }
    driveSubsystem.straightdrive(drive, -rotate); // motion control here with joystick throttle and rotation inputs
    //driveSubsystem.driveWithRotation(0.5, 0);
    //driveSubsystem.drive(-throttle, rotate);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

