// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final WPI_TalonSRX extend;
  private final WPI_TalonSRX rotate;

  public Arm() {
    extend = new WPI_TalonSRX(1);
    rotate = new WPI_TalonSRX(4);

    extend.setInverted(false);
    rotate.setInverted(false);

    extend.configFactoryDefault();
    rotate.configFactoryDefault();

    rotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    rotate.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotate.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
  }

  public void extendPow(double power){
    extend.set(ControlMode.PercentOutput, power);
  }

  public void rotatePow(double power){
    rotate.set(ControlMode.PercentOutput, power);
  }

  public double getArmTicks(){
    return rotate.getSelectedSensorPosition();
  }

  public void resetArm(){
    rotate.setSelectedSensorPosition(0, 0, 10);
  }

  public void setArmBrake(boolean brake){
    if(brake){
      rotate.setNeutralMode(NeutralMode.Brake);
    }else{
      rotate.setNeutralMode(NeutralMode.Coast);
    }
  }

  public int isFwdLSClosed(){
    return rotate.isFwdLimitSwitchClosed();
  }

  public int isRevLSClosed(){
    return rotate.isRevLimitSwitchClosed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("FwdLS:", isFwdLSClosed());
    SmartDashboard.putNumber("RevLS:", isRevLSClosed());

    SmartDashboard.putNumber("RotTicks:", getArmTicks());

    //extendPow(RobotContainer.getJoy1().getY());
    rotatePow(RobotContainer.getJoy1().getY());
  }
}
