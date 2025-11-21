package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Degree;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
    
    private Angle armAngle;

    private final SparkMax armMotorObject = new SparkMax(Constants.ArmContants.ARM_MOTOR_1, MotorType.kBrushless);
    private final AbsoluteEncoder armEncoderObject = armMotorObject.getAbsoluteEncoder();
    private final SparkMaxConfig armConfig = new SparkMaxConfig();

    private final PIDController pid = new PIDController(3, 0, 0);

    public ArmSubsystem()
    {
        armAngle = Angle.ofBaseUnits(0, Degree);

        armConfig.absoluteEncoder.positionConversionFactor(360);

        armMotorObject.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() 
    {
        armMotorObject.set(pid.calculate(armEncoderObject.getPosition(), armAngle.in(Degree)));
    }

    public void setTargetArmAngle(Angle targetAngle)
    {
        armAngle = targetAngle;
    }

    public Angle getTargetArmAngle()
    {
        return armAngle;
    }

}
