package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevating {
    private Elevator elevator;

    public Elevating(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command firstLevel() {
        return elevator.setLevel(Constants.ElevatorSetpoints.MIDDLE1);
    }

    public Command secondLevel() {
        return elevator.setLevel(Constants.ElevatorSetpoints.MIDDLE2);
    }

    public Command thirdLevel() {
        return elevator.setLevel(Constants.ElevatorSetpoints.TOP);
    }

    public Command resetLevel() {
        return elevator.setLevel(Constants.ElevatorSetpoints.BOTTOM)
            .until(elevator::hitBottomLimit)
            .andThen(elevator.stopElevatorMotorCmd());
    }
}
