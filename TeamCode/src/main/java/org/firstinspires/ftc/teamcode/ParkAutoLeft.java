package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/* Leave + Park with the Loading Zone to the robot's left (see ParkAuto). Then Mecanum Drive is queued up for TELEOP. */
@Autonomous(name = "Auto: Zone on my LEFT", group = "BioBuzz", preselectTeleOp = "Mecanum Drive")
public class ParkAutoLeft extends ParkAuto {
    @Override
    protected int zoneSide() {
        return -1;
    }
}
