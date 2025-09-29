package frc.robot.commands;
import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralTransition extends Command {
    Outtake outtake = new Outtake();

    public Command transition() {
        return outtake.runFoward()
            .until(outtake::isCoralInPosition)
            .andThen(outtake.stopMotor());
    }
}
