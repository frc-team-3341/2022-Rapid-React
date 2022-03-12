// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final class DriveTrainPorts
    {
        public static final int LeftDriveTalonPort = 2; 
        public static final int RightDriveTalonPort = 3;
        public static final int LeftDriveVictorPort = 4;
        public static final int RightDriveVictorPort = 5;
    }

    public static final class ArmPorts{
        public static final int FrontLeftArmRot = 6;  //encoder
        public static final int FrontRightArmRot = 7; //encoder
        public static final int BackLeftArmRot = 8;  //encoder
        public static final int BackRightArmRot = 9; //encoder

        public static final int FrontLeftArmExt = 10;
        public static final int FrontRightArmExt = 11;
        public static final int BackLeftArmExt = 12;
        public static final int BackRightArmExt = 13;
    }

    public static final class BallPorts{
        public static final int BallPivot = 14; //encoder
        public static final int BallIntake = 15; //victor
        public static final int BallLeftFly = 16; //encoder
        public static final int BallRightFly = 17; //encoder
    }

    public static final class JoystickAxis 
    {
        public static final int YAxis = 1;
        public static final int XAxis = 0;
    }

    public static class I2CAddresses {
        public static final int MaxbotixUltrasonicSensor = 112;
    } 
    public static final class USBOrder {
        public static final int Zero = 0;
        public static final int One = 1;

    }
}

