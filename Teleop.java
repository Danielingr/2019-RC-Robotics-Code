package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Teleop", group="Iterative Opmode")

public class Teleop extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private Servo servo0;
    private Servo servo1;
    private Servo servo2;
    private Servo servo3;


    double turntgtPower = 0;
    double tgtPower = 0;
    double armPower = 0;
    double clawPower = 0;
    double elbowPower = 0;
    @Override
    public void init() {
        motor0 = hardwareMap.get(DcMotor.class, "motor0");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }
    public void start() {
        motor0.setDirection(DcMotor.Direction.REVERSE);
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        runtime.reset();
    }
    @Override
    public void loop(){
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("motor 0",motor0.getCurrentPosition());
        telemetry.addData("motor 1",motor1.getCurrentPosition());
        telemetry.addData("motor 2",motor2.getCurrentPosition());
        telemetry.addData("motor 3",motor3.getCurrentPosition());
        telemetry.addData("servo8",gamepad2.right_stick_y);
        telemetry.addData("servo9",gamepad2.left_stick_y);

        tgtPower = this.gamepad1.left_stick_y;
        turntgtPower = this.gamepad1.right_stick_x/2;
        armPower = this.gamepad2.right_stick_y/2;
        clawPower = clawPower + this.gamepad2.left_stick_y;
        elbowPower = elbowPower + this.gamepad2.left_stick_y;
        double max = Math.max(Math.abs(tgtPower+turntgtPower),Math.abs(tgtPower-turntgtPower));
        double ls = tgtPower+turntgtPower;
        double rs = tgtPower-turntgtPower;
        if (max > .5)
        {
            ls = Range.scale(tgtPower+turntgtPower,-max,max,-0.5,.5);
            rs = Range.scale(tgtPower-turntgtPower,-max,max,-0.5,.5);
        }
        motor0.setPower(ls);
        motor1.setPower(rs);
        motor2.setPower(rs);
        motor3.setPower(ls);
        servo0.setPosition(clawPower);
        servo1.setPosition(elbowPower);


    }
    @Override
    public void init_loop() {
    }





    @Override
    public void stop() {
    }

}

