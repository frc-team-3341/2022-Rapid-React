// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class AutoPath extends SequentialCommandGroup 
{
    public AutoPath()
    {
        addCommands(new AutoDriveForward(RobotContainer.getDriveTrain(), 2.3));
    }
}
