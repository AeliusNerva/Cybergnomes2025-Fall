// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.signals.NeutralModeValue;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * hello
 */
public final class Constants {
//         public static final double STICK_DEAD_BAND = 0.1;
        
//    public static final class Swerve {

//         // Always ensure Gyro is CCW+ CW-
//         public static final boolean INVERT_GYRO = true;

//         private static final COTSTalonFXSwerveConstants chosenModule =
//                 COTSTalonFXSwerveConstants.SDS.MK4.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L2);

        

//         /* Drivetrain Constants */
//         // Distance from left wheel to right
//         private static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
//         // Distance from front wheel to back
//         private static final double WHEEL_BASE = Units.inchesToMeters(25);
//         public static final double WHEEL_CIRCUMFERENCE = chosenModule.wheelCircumference;

        //Coral/claw height constants
//public static final class ElevatorConstants{

       /*  public static final double kElevatorErrorTolerance = 0.5;

        public static final double StoreHeight = 0.0;
        public static final double L2Height = 9.0;
        public static final double L3Height = 25.14;
        public static final double L4Height = 52.0;
        public static final double MaxHeight = 56.2;*/

       // }
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        
        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        
   //PNEUMATICS
   public static final class Pneumatics {
        //public static final int CLAW_ID = ;
        public static final int CORALINTAKE_ID = 0;

        public static final int ALGAEINTAKE1_ID = 1;
        public static final int ALGAEINTAKE2_ID = 2;
        /*public static final int SHOOTER_ID = 12;
        public static final int PUSHER_ID = 13; */

        public static final double MIN_PRESSURE = 100;
        public static final double MAX_PRESSURE = 120;

    }

    //ELEVATOR
   public static final class Elevator {
       
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
        //Highest and lowest height that the elevator reaches
        public static final double ELEVATOR_MIN = -50;
        public static final double ELEVATOR_MAX = 0.2;

        public static final double ELEVATOR_POS = 0.8; //ADJUST THIS!!!!

        //Heights/Positions
        //public static final double MOVE_POSITION = -2;

        //public static final double L2Height = 9.0;
        public static enum ElevatorSetPosition{
                LEVEL_1_HEIGHT (0.5),
                LEVEL_2_HEIGHT (0.5),
                LEVEL_3_HEIGHT (1.5),
                LEVEL_4_HEIGHT (1.5),
                ZERO_HEIGHT (0);

                public final double position;

                ElevatorSetPosition(double setPosition) {
                        position = setPosition;
                }
        }

        public static final class LeftElevatorMotor{
                public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
                public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
                public static final int MOTOR_ID = 39; //Elevator.L
                public static final double ACCELERATION = 1;
                public static final double MAX_SPEED = 0.5;
                public static final double KP = 0.2;
                public static final double KI = 0;
                public static final double KD = 0;
        }

        public static final class RightElevatorMotor{
                public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;
                public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
                public static final int MOTOR_ID = 15; //Elevator.R
                public static final double ACCELERATION = 1;
                public static final double MAX_SPEED = 0.5;
                public static final double KP = 0.2;
                public static final double KI = 0;
                public static final double KD = 0;
        }
   }

   public static final class Algae {
        //Single falcon500 motor for algae intake
        public static final class AlgaeIntakeMotor {
                public static final int MOTOR_ID = 6; //FILL IN ID ONCE CONFIGURED!

        public static final double CANCODER_MIN = 0;
        public static final double CANCODER_MAX = 100;

        public static final class AlgaeIntake{
                public static final int MOTOR_ID = 6; 
                public static final double ACCELERATION = 6;
                public static final double MAX_SPEED = 6;
                public static final double KP = 10;
                public static final double KI = 10;
                public static final double KD = 10;
        }
        }
   }

   public static final class Claw {
        //single falcon500 motor for the claw to pivot
        
        public static double PIVOT_POS = 0.8;

        public static enum PivotSetPosition{
                UP(7), //CHANGE THESE ONCE TESTED!!!!!!!!!!!!
                OUT(1);


                public final double pivotPosition;

                PivotSetPosition(double setPivotPosition) {
                        pivotPosition = setPivotPosition;
                }
        }

        public static final class PivotMotor{
                public static final int MOTOR_ID = 7; //Pivot motor
                public static final double ACCELERATION = 6;
                public static final double MAX_SPEED = 6;
                public static final double KP = 0.1;
                public static final double KI = 0;
                public static final double KD = 0;

                public static final double CANCODER_MIN = 0.078; //FILL IN!!!
                public static final double CANCODER_MAX = 0.2; //FILL IN!!!

                //Threshold to determine if the pivot has fully rotated back (used for math check in pivot command)
                public static final double PIVOT_ANGLE_THRESHOLD = 1.0;
        
        }
        }
   

   public static final class VisionConstants {
        public static final double REEF_APRILTAG_HEIGHT = 0.6875;
        public static final double PROCCESSOR_APRILTAG_HEIGHT = 0.45875;
        public static final double CORAL_APRILTAG_HEIGHT = 0.5325;
   }

   public static final class Limelight {
        // height and angle of the right limelight
        public static final class Right {
            public static final String NAME = "limelight_2";

            public static double ANGLE = 0;
            public static double HEIGHT = 0.42008; 
        }
        // height and angle of the left limelight
        public static final class Left {
            public static final String NAME = "limelight_1";

            public static double ANGLE = 0;
            public static double HEIGHT = 0.42008;
        }

        public static final class Pipelines {
           
        }
    }

    

        /* Constraint for the motion profilied robot angle controller */
        
    }




