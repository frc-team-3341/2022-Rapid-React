// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.KeyPair;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RotatePID extends CommandBase {
  /** Creates a new RotatePID. */
  private double angle;
  private PIDController pid;
  private double kp = 0.005;
  private double ki = 0.0;
  private double kd = 0.0;

  public RotatePID(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getArm());
    this.angle = angle;
    pid = new PIDController(kp, ki, kd);
    pid.setTolerance(2, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidPower = pid.calculate(RobotContainer.getArm().getArmPosition());

    if(pidPower >= 0.5){
      pidPower = 0.5;
    }
    
    RobotContainer.getArm().rotatePow(pidPower);
    SmartDashboard.putNumber("RotatePIDPow:", pidPower);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getArm().rotatePow(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
