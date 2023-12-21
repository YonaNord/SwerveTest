package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Swerve;


public class SwerveModule extends SubsystemBase {
    // TODO https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/closed-loop-guide.html
    // TODO https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html

    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_steerEncoder;
    private final VelocityDutyCycle m_velocityDutyCycle;
    private final MotionMagicDutyCycle m_motionMagicDutyCycle;

    private SwerveModuleState m_moduleState;
    


    /**
     * @param driveMotorCANID
     * CANID of the Drive Motor (The Falcon motor that controls the wheel)
     * @param steerMotorCANID
     * CANID of the Steer Motor (The Falcon motor that controls turning)
     * @param steerEncoderCANID
     * CANID of the Steer Encoder (on-axis)
     * @param moduleOffsetInDegrees
     * The Offset of the module (Relative to the robot)
     */
    public SwerveModule(int driveMotorCANID, int steerMotorCANID, int steerEncoderCANID, double moduleOffsetInDegrees) {
        this.m_driveMotor = new TalonFX(driveMotorCANID);
        this.m_steerMotor = new TalonFX(steerMotorCANID);
        this.m_steerEncoder = new CANcoder(steerEncoderCANID);
        this.m_velocityDutyCycle = new VelocityDutyCycle(0);
        this.m_motionMagicDutyCycle = new MotionMagicDutyCycle(0);
        this.m_moduleState = new SwerveModuleState(0, Rotation2d.fromDegrees(moduleOffsetInDegrees));
        configMotors(steerEncoderCANID);
        setNeutralMode(NeutralModeValue.Brake);

    }

    /**
     * Adds voltage limits (safety), and true ratios to the motors
     * @param steerEncoderCANID
     * The CANID of the encoder that will replace the included encoder inside the Falcon 500
     */
    private void configMotors(int steerEncoderCANID) {
      // ! I DONT KNOW THE ACTUAL VOLTAGES AND STUFF, IM GUESSING BASED ON YOTAM'S FRC-2023-CODE ---------------------------------------

        VoltageConfigs voltageConfigs = new VoltageConfigs()
            .withPeakForwardVoltage(Swerve.Stats.kMaxVoltage)
            .withPeakReverseVoltage(Swerve.Stats.kMaxVoltage);

        CurrentLimitsConfigs statorConfigs = new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(35)
            .withSupplyCurrentLimit(35);

        FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(steerEncoderCANID)
            .withRotorToSensorRatio(Swerve.Stats.kRotorToSensorRatio);

        // Slot0Configs slot0DriveConfigs = new Slot0Configs()
        //     .withKA(Swerve.PID.Drive.kA) // Acceleration 
        //     .withKS(Swerve.PID.Drive.kS) // Static Friction Offset
        //     .withKV(Swerve.PID.Drive.kV) // Velocity Feedforward
        //     // P I D
        //     .withKP(Swerve.PID.Drive.kP)  // Proportional tuning - error
        //     .withKI(Swerve.PID.Drive.kI) // Integral tuning - learning
        //     .withKD(Swerve.PID.Drive.kD); // Derivative tuning - overshoot



        // Slot0Configs slot0SteerConfigs = new Slot0Configs()
        //     .withKA(Swerve.PID.Steer.kA) // Acceleration 
        //     .withKS(Swerve.PID.Steer.kS) // Static Friction Offset
        //     .withKV(Swerve.PID.Steer.kV) // Velocity Feedforward
        //     // P I D
        //     .withKP(Swerve.PID.Steer.kP)  // Proportional tuning - error
        //     .withKI(Swerve.PID.Steer.kI) // Integral tuning - learning
        //     .withKD(Swerve.PID.Steer.kD); // Derivative tuning - overshoot
        


        this.m_driveMotor.getConfigurator().apply(voltageConfigs);
        this.m_driveMotor.getConfigurator().apply(statorConfigs);
        // this.m_driveMotor.getConfigurator().apply(slot0DriveConfigs);
        
        this.m_steerMotor.getConfigurator().apply(voltageConfigs);
        this.m_steerMotor.getConfigurator().apply(statorConfigs);
        this.m_steerMotor.getConfigurator().apply(feedbackConfigs);
        // this.m_steerMotor.getConfigurator().apply(slot0SteerConfigs);
        

        
    }


    /**
     * @param state 
     * The state of the Module (Affects both the Drive and Steer Motor)
     */
    public void setModuleState(SwerveModuleState state){
        this.m_moduleState = SwerveModuleState.optimize(state, this.m_moduleState.angle);
        this.m_driveMotor.setControl(this.m_velocityDutyCycle.withVelocity(mpsToRpm(state.speedMetersPerSecond)));
        this.m_steerMotor.setControl(this.m_motionMagicDutyCycle.withPosition(state.angle.getDegrees()));
    }

    /**
     * Convertion from Meters per second to Rounds per minute 
     * @param value 
     * Rounds per minute
     */
    public double mpsToRpm(double value) {
        double wheelRadiusMeters = Units.inchesToMeters(Swerve.Stats.kDriveWheelRadiusInches);
        return (60/(2*Math.PI*wheelRadiusMeters))*value;
    }   

    /**
     * @param neutralMode
     * Set's the NeutralMode of both motors of the module to CTRE'S NeutralMode
     * 
     * <i>(Check CTRE's NeutralMode documentation for more info)</i>
     *  
     */

    

    public TalonFX getDriveMotor() {
        return this.m_driveMotor;
    }

    public TalonFX getSteerMotor() {
        return this.m_steerMotor;
    }

    public CANcoder getSteerEncoder() {
        return this.m_steerEncoder;
    }

    public SwerveModuleState getModuleState() {
        return this.m_moduleState;
    }

    /**
     * @param neutralMode
     * The NeutralMode for the SwerveModule's motors
     */
    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.m_driveMotor.setNeutralMode(neutralMode);
        this.m_steerMotor.setNeutralMode(neutralMode);
    }

    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and 1.0.
     */
    public void setDriveMotor(int speed) {
        
        this.m_driveMotor.set(speed);
    }


    /**
     *
     * @param speed The speed to set (Percentage). Value should be between -1.0 and 1.0.
     */
    public void setSteerMotor(int speed) {
        this.m_steerMotor.set(speed);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
