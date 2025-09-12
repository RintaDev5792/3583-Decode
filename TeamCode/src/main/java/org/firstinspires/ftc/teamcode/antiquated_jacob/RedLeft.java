package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.concurrent.TimeUnit;

@Autonomous( name = "RED-LEFT")

public class RedLeft extends Skely{
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        runtime.reset();
        zeroClaws();
        runMHAuto(1,2);

    }
    public void zeroClaws(){
        clawLeft.setPosition(0);
        clawRight.setPosition(1);
        clawRotate.setPosition(0.25);

    }


}

