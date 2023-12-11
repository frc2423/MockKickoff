package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.devices.NeoMotor;

//we are children dont judge us :P

public class Intake {
    public static NeoMotor intakeRollerMotor = new NeoMotor (7);    
    private double rollerSpeed = 0.50;

    private boolean isDown = false;
    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);

    public void intakeUp() {
        intakeRollerMotor.setPercent(0);
        solenoidUp();
    }

    public void intakeDown() {
        intakeRollerMotor.setPercent(0.50);
        solenoidDown();
    }
    
    public boolean isDown() {
        return isDown;
    }

    private void solenoidUp() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    
    }
    private void solenoidDown() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void runIntake() {
         if (!isDown()) {
            intakeUp();
        } else {
            intakeDown();
        }
    }

}