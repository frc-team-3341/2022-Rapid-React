// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
  

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {
  /** Creates a new TankDrive. */
  private final DriveTrain _driveTrain;
  private final Joystick _Joystick;

  public ArcadeDrive(DriveTrain dt, Joystick jt) {
    // Use addRequirements() here to declare subsystem dependencies.
    _driveTrain = dt;
    _Joystick = jt;

    addRequirements(_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("ArcadeDrive");
    _driveTrain.arcadeDrive(0.8 * _Joystick.getRawAxis(Constants.JoystickAxis.XAxis),
      0.8 * _Joystick.getRawAxis(Constants.JoystickAxis.YAxis));
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
