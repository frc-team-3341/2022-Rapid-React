// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.MaxbotixUltrasonicSensor;

import frc.robot.commands.ArmExtend;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static Limelight limelight;
  private static Arm arm;
  private static ArmExtend extend;

  public static Joystick joy1;
  public static JoystickButton redPipeline;
  public static JoystickButton bluePipeline;
  public static JoystickButton rotate20;


  private static Joystick _leftJoystick;
  private static Joystick _rightJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    //limelight = new Limelight();
    arm = new Arm();
    //test = new testAccumulate();
    //rotatePID = new RotatePID(20);
    extend = new ArmExtend(4);

    joy1 = new Joystick(0);
    //rotate20 = new JoystickButton(joy1, 1);
    //redPipeline = new JoystickButton(joy1, 3);
    //bluePipeline = new JoystickButton(joy1, 4);

    _leftJoystick = new Joystick(Constants.USBOrder.Zero);
    _rightJoystick = new Joystick(Constants.USBOrder.One);
    //_DriveTrain = new DriveTrain();
    
    //_tankDrive = new TankDrive(_DriveTrain, _leftJoystick, _rightJoystick);
    //_arcadeDrive = new ArcadeDrive(_DriveTrain, _leftJoystick);
    //_DriveTrain.setDefaultCommand(_arcadeDrive);
    
    //driveForward = new AutoDriveForward(_DriveTrain, -100);
    //autoPath = new AutoPath();
    //test = new testTicksToCm(_DriveTrain);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return extend;
  }

  /*public static DriveTrain getDriveTrain(){
    return _DriveTrain;
  }*/

  public static Limelight getLimelight(){
    return limelight;
  }

  public static Arm getArm(){
    return arm;
  }

  public static Joystick getLeftJoy(){
    return _leftJoystick;
  }

  public static Joystick getRightJoy(){
    return _rightJoystick;
  }

  public static Joystick getJoy(){
    return joy1;
  }

}
