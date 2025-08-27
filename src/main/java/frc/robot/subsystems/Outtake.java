package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Ports.OuttakePorts;
import frc.robot.Constants;

public class Outtake extends SubsystemBase {
    private final CANSparkMax outtakeMotor;
    private final DigitalInput frontBeamBreak;
    private final DigitalInput middleBeamBreak;

    public Outtake() {
        outtakeMotor = new CANSparkMax(Ports.OuttakePorts.OUTTAKE_MOTOR, MotorType.kBrushless);
        frontBeamBreak = new DigitalInput(OuttakePorts.FRONT_BEAM_BREAK);
        middleBeamBreak = new DigitalInput(OuttakePorts.MIDDLE_BEAM_BREAK);
    }
}
