package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.PneumaticsHandler;
import frc.robot.RobotContainer;
import frc.robot.commands.Claw.PivotCommand;

public class ElevatorLevelCommand extends SequentialCommandGroup {
    private static double heightAdjustment = 0; // Adjustable height offset
    private static double descentAdjustment = 0; // Adjustable descent offset

    public ElevatorLevelCommand(Constants.Elevator.ElevatorSetPosition targetPosition) {
        addCommands(
            // Activate coral intake and wait 2 seconds
            new InstantCommand(() -> RobotContainer.h_pneumatics.setCoralPusherSolenoid(true)),
            new WaitCommand(2),
            // Move claw pivot to OUT position
            new PivotCommand(Constants.Claw.PivotSetPosition.OUT),
            new WaitCommand(2),
            // Open the claw
            new InstantCommand(() -> RobotContainer.h_pneumatics.setClawSolenoid(true)),
            new WaitCommand(2),
            // Deactivate coral intake and wait 5 seconds
            new InstantCommand(() -> RobotContainer.h_pneumatics.setCoralPusherSolenoid(false)),
            new WaitCommand(5),
            // Close the claw
            new InstantCommand(() -> RobotContainer.h_pneumatics.setClawSolenoid(false)),
            new WaitCommand(2),
             // Move claw pivot to UP position
             new PivotCommand(Constants.Claw.PivotSetPosition.UP),
             new WaitCommand(2),
            // Move elevator to the target position with height adjustment using PID
            new InstantCommand(() -> RobotContainer.s_Elevator.setPosition(targetPosition.position + heightAdjustment)),
            new WaitCommand(2),
           
            // Slowly move elevator down by descent adjustment using PID
            new InstantCommand(() -> RobotContainer.s_Elevator.setPosition(targetPosition.position + heightAdjustment - descentAdjustment)),
            new WaitCommand(2),
            // Move claw pivot back to OUT position
            new PivotCommand(Constants.Claw.PivotSetPosition.OUT),
            new WaitCommand(2),
            // Move elevator to ZERO position using PID
            new InstantCommand(() -> RobotContainer.s_Elevator.setPosition(Constants.Elevator.ElevatorSetPosition.ZERO_HEIGHT.position)),
            new WaitCommand(2)
        );
    }

    public static void adjustHeight(double adjustment) {
        heightAdjustment += adjustment;
    }

    public static void adjustDescent(double adjustment) {
        descentAdjustment += adjustment;
    }
}
