package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumIMU extends OpMode {
    private DcMotor leftFront,leftBack,rightFront,rightBack;
    private IMU imu;
    PIDcontrolador pid;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        pid = new PIDcontrolador(0,0,0);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
    }

    @Override
    public void loop() {
         Mecanum();
    }
    public void Mecanum(){
        double X = gamepad1.left_stick_x;
        double Y = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(Y) + Math.abs(X) + Math.abs(rx), 1);

        if (Math.abs(rx) > 0.1) {

            double anguloAlvo = 0;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double correcao = pid.calculo(anguloAlvo, botHeading);

            if (correcao > 180) {
                correcao -= 360;
            } else if (correcao < -180) {
                correcao += 360;
            }

            double rotX = X * Math.cos(-botHeading) - Y * Math.sin(-botHeading);
            double rotY = X * Math.sin(-botHeading) + Y * Math.cos(-botHeading);

            double frontLeftPower = (Y + X + rx + correcao) / denominator;
            double backLeftPower = (Y - X + rx + correcao) / denominator;
            double frontRightPower = (Y - X - rx - correcao) / denominator;
            double backRightPower = (Y + X - rx- correcao) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }else {
            double frontLeftPower = (Y + X + rx ) / denominator;
            double backLeftPower = (Y - X + rx) / denominator;
            double frontRightPower = (Y - X - rx ) / denominator;
            double backRightPower = (Y + X - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
        }


    }
}
