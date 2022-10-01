package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.command.RunEndCommand;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The intake subsystem, which extends and spins to pull balls over the bumper
 * into the midtake.
 * 
 * @author Ben Su, Jeremiah Shue
 */
public class IntakeS extends SubsystemBase implements Loggable {
    @Log(methodName = "getAppliedOutput", name = "inputVolts")
    private final CANSparkMax intakeLeadMotor = new CANSparkMax(Constants.CAN_ID_INTAKE_LEAD_MOTOR,
            MotorType.kBrushless);
    private final CANSparkMax intakeFollowerMotor = new CANSparkMax(Constants.CAN_ID_INTAKE_FOLLOWER_MOTOR,
            MotorType.kBrushless);
    private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.DOUBLE_SOLENOID_INTAKE_PORT_EXTEND, Constants.DOUBLE_SOLENOID_INTAKE_PORT_RETRACT);

    /**
     * Constructs a new IntakeS.
     */
    public IntakeS() {
        retract();
        intakeLeadMotor.restoreFactoryDefaults();
        intakeFollowerMotor.restoreFactoryDefaults();
        intakeFollowerMotor.follow(intakeLeadMotor, true);
    }

    /**
     * Extends the intake.
     */
    public void deploy() {
        doubleSolenoid.set(Value.kForward);
    }

    /**
     * Retracts the intake.
     */
    public void retract() {
        doubleSolenoid.set(Value.kReverse);
    }

    /**
     * Toggles the intake.
     */
    public void toggle() {
        doubleSolenoid.toggle();
    }

    /**
     * Spins the intake at the given speed.
     * 
     * @param speed the speed
     */
    public void spin(double speed) {
        intakeLeadMotor.setVoltage(speed * 12);
    }

    /**
     * Spins the intake inward at a speed set in Constants.
     */
    public void spin() {
        spin(Constants.INTAKE_SPEED);
    }

    /**
     * Spins the intake outward at a speed set in Constants to eject balls.
     */
    public void eject() {
        spin(Constants.INTAKE_EJECT_SPEED);
    }

    /**
     * Stops the intake.
     */
    public void stop() {
        spin(0);
    }

    @Override
    public void periodic() {
    }

    /**
     * Returns a RunCommand that spins the intake and stops it on command end.
     * @return the spin command.
     */
    public Command spinC() {
        return new RunEndCommand(this::spin, this::stop, this);
    }

    /**
     * Returns a RunCommand that deploys the intake.
     * @return the deploy command.
     */
    public Command deployC() {
        return new RunCommand(this::deploy, this);
    }

    /**
     * Returns a RunCommand that deploys the intake.
     * @return the deploy command.
     */
    public Command retractC() {
        return new RunCommand(this::retract, this);
    }

    public Command deployAndSpinC() {
        return new RunEndCommand(
            ()-> {
                this.deploy();
                this.spin();
            }, 
            ()-> {
                this.retract();
                this.stop();
            }, this);
    }



}
