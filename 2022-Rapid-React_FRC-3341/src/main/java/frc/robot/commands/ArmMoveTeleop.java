// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arm1;

public class ArmMoveTeleop extends CommandBase {
  /** Creates a new ArmMoveTeleop. */
  private Arm1 armSub;

  public ArmMoveTeleop(Arm1 armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    armSub = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      /*
        POV + trigger control extension/retraction (POV designates direction and trigger sets power)
        Y-axis controls rotation
      */
      SmartDashboard.putString("armNum Command", armSub.getName());
      double joyY = RobotContainer.getJoy1().getY();
      double joyX = RobotContainer.getJoy1().getX();
      double POV = RobotContainer.getJoy1().getPOV();

      if((POV >= 0 && POV < 70) || (POV >= 250)){
        if(RobotContainer.getJoy1().getRawButtonPressed(1)){
          //power level will be changed with testing
          armSub.extend(0.1);
        }
      }
      else if(POV >= 110 && POV <= 250){
        if(RobotContainer.getJoy1().getRawButtonPressed(1)){
          //power level will be changed with testing
          armSub.extend(-0.1);
        }
      }

      armSub.rotate(joyY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSub.extend(0);
    armSub.rotate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
