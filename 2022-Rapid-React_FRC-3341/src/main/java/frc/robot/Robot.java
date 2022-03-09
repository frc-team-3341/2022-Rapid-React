// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  WPI_TalonSRX left = new WPI_TalonSRX(2), right = new WPI_TalonSRX(3);
  WPI_TalonSRX leftFlywheel = new WPI_TalonSRX(16), rightFlywheel = new WPI_TalonSRX(17);
  WPI_VictorSPX intake = new WPI_VictorSPX(15);

  private Timer time;
  private static double analogValue;
  private AnalogInput input;
  private double currPos;
  private double prevTime;

  private double armpower;
  private int armExtPos;
  private boolean armExtPrevState;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    time = new Timer();
    time.reset();
    prevTime = time.getFPGATimestamp();
    input = new AnalogInput(0);
    currPos = 0;
    armExtPos = 0;
    armExtPrevState = isOnTape();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //RobotContainer.getLimelight().update();
    
    addPeriodic(() -> {
      analogValue = input.getValue();
      SmartDashboard.putNumber("analog input", analogValue);
     // m_robotContainer.getArm().armCount();
      SmartDashboard.putBoolean("Sensor:", isOnTape());
      SmartDashboard.putNumber("ArmExtPos:", armExtPos);
      armCount();

      double currentTime = time.getFPGATimestamp();
      SmartDashboard.putNumber("Delta T", currentTime - prevTime);
      prevTime = currentTime;

      /*
      if(lineNum > currPos){
        RobotContainer.getArm().extendPow(0.3);
      } else if(lineNum < currPos){
        RobotContainer.getArm().extendPow(-0.3);
      }
      */
    }, 0.02, 0.01
    );
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() 
  {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
    left.set(ControlMode.PercentOutput, -1*RobotContainer.getLeftJoy().getY());
    right.set(ControlMode.PercentOutput, RobotContainer.getRightJoy().getY());
    if (RobotContainer.getLeftJoy().getRawButton(1)) {
      leftFlywheel.set(ControlMode.PercentOutput, 0.7);
      rightFlywheel.set(ControlMode.PercentOutput, -0.7);
      intake.set(ControlMode.PercentOutput, -0.7);
    } else {
      leftFlywheel.set(ControlMode.PercentOutput, 0);
      rightFlywheel.set(ControlMode.PercentOutput, 0);
      intake.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  public static double analogValue(){
    return analogValue;
  }

  public static boolean isOnTape(){
    return analogValue > 1000;
  }

  public void armCount(){
    /*
    if(m_robotContainer.getArm().getArmPower() < 0){
      if(!m_robotContainer.getArmExtPrevState() && isOnTape()){
          m_robotContainer.negArmExtPos();
      }
    }else if(m_robotContainer.getArm().getArmPower()  > 0){
      if(!m_robotContainer.getArmExtPrevState() && isOnTape()){
          m_robotContainer.addArmExtPos();;
    }
    */
    
    if(!armExtPrevState && isOnTape()){
      armExtPos++;
    }
    armExtPrevState = isOnTape(); 
  }
  
}
