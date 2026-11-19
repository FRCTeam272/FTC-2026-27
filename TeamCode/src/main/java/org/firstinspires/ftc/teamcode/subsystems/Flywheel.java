package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.Context;

public final class Flywheel {
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(1.16, 0, 0, 19.6);
    public static int velocityTolerance = 50;
    private final DcMotorEx flywheelMotor;

    private VoltageSensor batteryVoltageSensor;
    private final Context context;
    private Mode mode = Mode.OFF;
    private double target = 0; //was 1500, put at 0 once there is a shooting controller
    private double manualAdjust = 0;
    private double MAX_VELOCITY = 1500; // max allowed target velocity

    public Flywheel(Context context) {
        this.context = context;

        // ================== MOTORS ================== \\
        flywheelMotor = context.motor("flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorConfigurationType MotorConfigurationType = flywheelMotor.getMotorType().clone();
        MotorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotor.setMotorType(MotorConfigurationType);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        batteryVoltageSensor = context.hardwareMap.voltageSensor.iterator().next();
        setPIDFCoefficients(flywheelMotor, MOTOR_VELO_PID);
    }

    public void setTarget(double target) {
        if (target + manualAdjust > MAX_VELOCITY) {
            this.target = MAX_VELOCITY;
        } else {
        this.target = target + manualAdjust; }
    }

    public void setManualAdjust(double adjustment) {
        this.manualAdjust = manualAdjust + adjustment;
    }

    // Zeroes out manualAdjust, just in case it gets out of control
    public void zeroManualAdjust() {
        this.manualAdjust = 0;
    }

    public void off() {
        mode = Mode.OFF;
    }

    public void on() {
        mode = Mode.VELOCITY;
    }

    public void toggle() {
        mode = mode == Mode.OFF ? Mode.VELOCITY : Mode.OFF;
    }

    public double getVelocity() {
        return flywheelMotor.getVelocity();
    }

    public double getTarget() {
        return target;
    }

    public double getManualAdjust() {
        return manualAdjust;
    }

    public boolean atTarget() {
        return Math.abs(target - getVelocity()) <= velocityTolerance;
    }

    private void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case OFF:
                    flywheelMotor.setVelocity(0);
                    return;
                case VELOCITY:
                    setPIDFCoefficients(flywheelMotor, MOTOR_VELO_PID);
                    flywheelMotor.setVelocity(target);
                    break;
            }

            context.telemetry.addData("Flywheel/velocity", getVelocity());
            context.telemetry.addData("Flywheel/target", target);
            context.telemetry.addData("Flywheel/manualAdjust", manualAdjust);

        });
    }

    enum Mode {
        OFF,
        VELOCITY
    }
}
