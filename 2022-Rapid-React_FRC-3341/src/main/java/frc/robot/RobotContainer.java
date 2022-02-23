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
import frc.robot.subsystems.DriveTrain;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.TankDrive;
import frc.robot.commands.AutoPath;
import frc.robot.commands.AutoDriveForward;
import frc.robot.commands.TurnGyroPID;
import frc.robot.commands.testTicksToCm;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static MaxbotixUltrasonicSensor ultrasonicSensor;
  private static Limelight limelight;
  private static Arm arm;

  public static Joystick joy1;
  public static JoystickButton redPipeline;
  public static JoystickButton bluePipeline;

  private static DriveTrain _DriveTrain;
  
  private final Joystick _leftJoystick;
  private final Joystick _rightJoystick;

  private static AutoDriveForward driveForward;
  private static testTicksToCm test;

  private final TankDrive _tankDrive;
  private final ArcadeDrive _arcadeDrive;

  private static AutoPath autoPath;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    ultrasonicSensor = new MaxbotixUltrasonicSensor(Constants.I2CAddresses.MaxbotixUltrasonicSensor);
    limelight = new Limelight();
    arm = new Arm();

    joy1 = new Joystick(0);
    redPipeline = new JoystickButton(joy1, 3);
    bluePipeline = new JoystickButton(joy1, 4);

    _leftJoystick = new Joystick(Constants.USBOrder.Zero);
    _rightJoystick = new Joystick(Constants.USBOrder.One);
    _DriveTrain = new DriveTrain();
    
    _tankDrive = new TankDrive(_DriveTrain, _leftJoystick, _rightJoystick);
    _DriveTrain.setDefaultCommand(_tankDrive);
    _arcadeDrive = new ArcadeDrive(_DriveTrain, _leftJoystick);
    driveForward = new AutoDriveForward(_DriveTrain, -100);
    autoPath = new AutoPath();
    test = new testTicksToCm(_DriveTrain);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return test;
  }

  public static DriveTrain getDriveTrain(){
    return _DriveTrain;
  }

  public static Limelight getLimelight(){
    return limelight;
  }

  public static Joystick getJoy1(){
    return joy1;
  }
}
