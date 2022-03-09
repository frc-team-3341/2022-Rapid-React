// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final WPI_TalonSRX extend;
  private final WPI_TalonSRX rotate;
 // private final AnalogInput input;

  //arm variables
  private int armExtPos;
  private double armPower;
  private boolean armExtPrevState;
  private final int minArmState = 0;
  private final int maxArmState = 5;
  private Timer time;

  private double previousTime = 0;

  public Arm() {
    //input = new AnalogInput(0);
    armExtPos = 0;
    armPower = 0;
    armExtPrevState = isOnTape();

    if(armExtPrevState) armExtPos++; 

    extend = new WPI_TalonSRX(1);
    rotate = new WPI_TalonSRX(4);

    extend.setInverted(false);
    rotate.setInverted(false);

    extend.configFactoryDefault();
    rotate.configFactoryDefault();

    rotate.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
    rotate.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    rotate.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    time = new Timer();
    time.reset();
  }

  public boolean isOnTape(){
    return Robot.analogValue() > 1000;
  }

  public void extendPow(double power){
    extend.set(ControlMode.PercentOutput, power);
    armPower = power;
  }

  public void rotatePow(double power){
    rotate.set(ControlMode.PercentOutput, power);
  }

  public double getArmTicks(){
    return rotate.getSelectedSensorPosition();
  }

  public double getArmPosition(){
    return getArmTicks() * 360 / 4096.0;
  }

  public double getArmPower(){
    return armPower;
  }

  public void resetArm(){
    rotate.setSelectedSensorPosition(0, 0, 10);
  }

  public double getArmExtCurrent(){
    return extend.getSupplyCurrent();
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

  public void armCount(){
    if(armPower < 0){
      if(!armExtPrevState && isOnTape()){
          armExtPos--;
      }
    }else if(armPower > 0){
      if(!armExtPrevState && isOnTape()){
          armExtPos++;
    }
    }
    armExtPrevState = isOnTape();
  }

  public int getArmExtPos(){
    return armExtPos;
  }

  public int getArmMaxPos(){
    return maxArmState;
  }

  public int getArmMinPos(){
    return minArmState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("FwdLS:", isFwdLSClosed());
    //SmartDashboard.putNumber("RevLS:", isRevLSClosed());
    //SmartDashboard.putNumber("ArmAnalogInput", Robot.analogValue());
    //SmartDashboard.putNumber("ArmPower:", armPower);

    if (isRevLSClosed() == 0){
      resetArm();  
    }
    
    
    //armCount();

    extendPow(RobotContainer.getJoy().getY() * 0.5);
    /*double currentTime = time.getFPGATimestamp();
    SmartDashboard.putNumber("Time:", currentTime);
    SmartDashboard.putNumber("Delta T", currentTime - previousTime);
    previousTime = currentTime;*/
    //rotatePow(RobotContainer.getJoy1().getY() * 0.3);

    /*
      afternoon sun: black tape:~1700-2000 bar: 120
      flashlight:
    */


  }
}
