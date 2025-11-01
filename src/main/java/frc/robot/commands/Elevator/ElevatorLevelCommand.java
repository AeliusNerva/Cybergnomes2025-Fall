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
            // Move claw pivot to UP position
            new PivotCommand(Constants.Claw.PivotSetPosition.UP),
            // Activate coral intake and wait 2 seconds
            new InstantCommand(() -> RobotContainer.h_pneumatics.setCoralPusherSolenoid(true)),
            new WaitCommand(2),
            // Close the claw
            new InstantCommand(() -> RobotContainer.h_pneumatics.setClawSolenoid(false)),
            // Move elevator to the target position with height adjustment
            new InstantCommand(() -> RobotContainer.s_Elevator.setPosition(targetPosition.position + heightAdjustment)),
            // Move claw pivot to DOWN position
            new PivotCommand(Constants.Claw.PivotSetPosition.OUT),
            // Slowly move elevator down by descent adjustment
            new InstantCommand(() -> RobotContainer.s_Elevator.setPosition(targetPosition.position + heightAdjustment - descentAdjustment)),
            // Move claw pivot back to UP position
            new PivotCommand(Constants.Claw.PivotSetPosition.UP),
            // Move elevator to ZERO position
            new InstantCommand(() -> RobotContainer.s_Elevator.setPosition(Constants.Elevator.ElevatorSetPosition.ZERO_HEIGHT.position))
        );
    }

    public static void adjustHeight(double adjustment) {
        heightAdjustment += adjustment;
    }

    public static void adjustDescent(double adjustment) {
        descentAdjustment += adjustment;
    }
}
