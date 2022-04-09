// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class BallShoot extends CommandBase {
  /** Creates a new BallShoot. */
  private double angle;
  private boolean isShooting;
  private boolean isTimerReset;
  private double origTime, timeCurr;
  private Timer time;

  public BallShoot(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angle;
    isShooting = false;
    isTimerReset = false;
    origTime = 0.0;
    timeCurr = 0.0;
    time = new Timer();
    addRequirements(RobotContainer.getBallHandler());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTimerReset = false;
    RobotContainer.getBallHandler().pidReset();
    RobotContainer.getBallHandler().getPIDController().setI(0.0005);
    RobotContainer.getBallHandler().setPivotAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isTimerReset){
      origTime = time.getFPGATimestamp();
      isTimerReset = true;
    }

    timeCurr = time.getFPGATimestamp() - origTime; 
    //SmartDashboard.putNumber("time:", timeCurr);

      RobotContainer.getBallHandler().setAnglePower();

      RobotContainer.getBallHandler().setFlywheelPower(1.0);
    
      //SmartDashboard.putNumber("PivotPosError:", RobotContainer.getBallHandler().getPositionError());
      if(timeCurr >= 3){
          RobotContainer.getBallHandler().setRollerPower(1.0);
          SmartDashboard.putNumber("RollerPower", 1.0);
      } 
      /*else if (timeCurr > 1) {
        RobotContainer.getBallHandler().setRollerPower(0.0);
        SmartDashboard.putNumber("RollerPower", 0.0);
      } */
      else{
        RobotContainer.getBallHandler().setRollerPower(-1.0);
        SmartDashboard.putNumber("RollerPower", -1.0);
      }
  
    SmartDashboard.putBoolean("isShooting", isShooting);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.getBallHandler().getPIDController().setI(0.00);
    RobotContainer.getBallHandler().setFlywheelPower(0.0);
    RobotContainer.getBallHandler().setRollerPower(0.0);
    SmartDashboard.putNumber("RollerPower", 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeCurr > 5;
  }
}
