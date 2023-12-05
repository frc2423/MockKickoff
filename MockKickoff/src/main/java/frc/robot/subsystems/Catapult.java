package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Catapult {

    private DigitalInput limitSwitch = new DigitalInput(0);
    private NeoMotor shoulderMotor;
    public static final double DISTANCE = 0;
    private double SHOULDER_MAXIMUM = 125; // calculate later :)
    private double SHOULDER_SHOOTING_MAXIMUM = 90; //not my problem, its not real
    private CANCoder shoulderEncoder = new CANCoder(25);
    CANCoderConfiguration _canCoderConfiguration = new CANCoderConfiguration();
    ProfiledPIDController shoulder_PID = new ProfiledPIDController((Robot.isSimulation()) ? .001 : .005, 0, 0, new TrapezoidProfile.Constraints(360, 420));//noice
    private ArmFeedforward feedforward = new ArmFeedforward(0.16623, kg, 17.022, 1.7561);
    private double shoulderVoltage = 0;
    public static final double MAX_SHOULDER_VOLTAGE = 4;
    private Rotation2d shoulderSetpoint = new Rotation2d();
    private Rotation2d shoulderAngle = new Rotation2d(0);

    public Catapult() {
        shoulderMotor = new NeoMotor(15, true);
        _canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        _canCoderConfiguration.magnetOffsetDegrees = -135;
        shoulderEncoder.configAllSettings(_canCoderConfiguration);
        shoulder_PID.setTolerance(5);
    }

    // moves the shoulder angle towards the front of the robot
    // limit shoulder angle
    public void shoulderForward() {
        setShoulderSetpoint(shoulderSetpoint.plus(Rotation2d.fromDegrees(5)));
    }

    // moves the shoulder angle towards the back of the robot
    // limit shoulder angle
    public void shoulderBack() {
        setShoulderSetpoint(shoulderSetpoint.minus(Rotation2d.fromDegrees(5)));
    }

    private void setShoulderVoltage(double voltage) {
        shoulderVoltage = voltage;
    }

    private void setShoulderVelocity(double radiansPerSecond) {
        double voltage = feedforward.calculate(getShoulderAngle().getRadians() + (Math.PI / 2), radiansPerSecond);
        setShoulderVoltage(voltage);
    }

    public void shoulderStop() {
        setShoulderSetpoint(getShoulderAngle());
    }
    // this function is to give a certain degrees int and this function will set the
    // motor to the desired location.

    public void setShoulderSetpoint(Rotation2d shoulderAngle) { // to spin comment out the if statement around ahoulder
                                                                // setpoint stuff
        if (shoulderAngle.getDegrees() >= -SHOULDER_MAXIMUM && shoulderAngle.getDegrees() <= SHOULDER_MAXIMUM) {
            shoulderSetpoint = shoulderAngle;
        }
    }

    public void resetShoulder() {
        shoulderEncoder.setPosition(0);
    }

    public Rotation2d getShoulderAngle() {
        return shoulderAngle;
    }

    public double getShoulderEncoderPosition() {
        return shoulderEncoder.getAbsolutePosition();
    }

    private double calculatePid(Rotation2d angle) {
        return shoulder_PID.calculate(shoulderAngle.getDegrees(), angle.getDegrees());
    }

    public void realPeriodic(double shoulderMotorPercent) {
        // Update real robot inputs
        shoulderMotor.setPercent(-shoulderMotorPercent);
        shoulderAngle = Rotation2d.fromDegrees(shoulderEncoder.getAbsolutePosition());
    }

    public void periodic() {
        double shoulderMotorPercent = shoulderMotor.getPercent();
        setShoulderVelocity(calculatePid(shoulderSetpoint));
        if (shoulderAngle.getDegrees() >= getMaxShoulderAngle() && shoulderVoltage > 0) {
            shoulderMotorPercent = 0;
        } else if (shoulderAngle.getDegrees() <= -getMaxShoulderAngle() && shoulderVoltage < 0) {
            shoulderMotorPercent = 0;
        } else {
            var voltage = MathUtil.clamp(shoulderVoltage, -MAX_SHOULDER_VOLTAGE, MAX_SHOULDER_VOLTAGE);
            shoulderMotorPercent = (voltage / RobotController.getBatteryVoltage());
        }
    }

    public double getMaxShoulderAngle() {
        return SHOULDER_MAXIMUM;
    }

    public Rotation2d getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }
    
    
    public void shoot(double speedInRadians){
        setShoulderSetpoint(shoulderAngle);
        if (shoulderAngle.getDegrees() >= SHOULDER_SHOOTING_MAXIMUM){
            setShoulderVelocity(0);
        }
        else{
            setShoulderVelocity(speedInRadians);
        }
    }
}
