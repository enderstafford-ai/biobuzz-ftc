# BioBuzz 38196 robot code

Everything our team wrote lives in this one folder. The rest of the project is the standard FTC SDK (12.0).

## The robot

goBILDA 4-motor mecanum drive (belt-driven wheels), REV Control Hub + Expansion Hub, goBILDA Pinpoint odometry.

| Name in the robot configuration | What it is | Plugged into |
|---|---|---|
| `frontLeft`, `backLeft`, `frontRight`, `backRight` | drive motors | Control Hub motor ports 0, 1, 2, 3 |
| `pinpoint` | goBILDA Pinpoint Odometry Computer (2 pods + gyro) | Control Hub I2C bus 0 |
| `imu` | the gyro inside the Control Hub (backup) | built in |
| `intake`, `intake2` | intake motors, both ends of one axle | Expansion Hub motor ports 0 and 2 |
| `flywheel` | shooter, 6000 RPM motor + encoder cable | Expansion Hub motor port 1 |
| `brake` | servo that pushes the brake pads down | Expansion Hub servo port 0 |
| `roller1`, `roller2` | continuous-rotation servos | Control Hub servo 0, Expansion Hub servo 1 |
| `Webcam 1` | Logitech C270 (Camera Test only) | Control Hub USB |

## Op modes (what shows up on the Driver Hub)

| Op mode | File | What it's for |
|---|---|---|
| **Mecanum Drive** | `MecanumDrive.java` | The match TeleOp. Start here. |
| Auto: Zone on my LEFT / RIGHT | `ParkAuto.java` (+ `ParkAutoLeft`, `ParkAutoRight`) | Autonomous: leave the wall, park in our Loading Zone. |
| Wheel Test | `WheelTest.java` | Step-by-step check of the drivetrain and sensors. Run after any rebuild. |
| Intake Test | `IntakeTest.java` | Makes every intake motor and roller turn the IN way, one at a time. |
| Camera Test | `CameraTest.java` | Reads the AprilTags. Nothing moves. |
| Simple Drive | `SimpleDrive.java` | Last year's drive math, for comparing. |

## Controls in Mecanum Drive

| While driving | | On the INIT screen (saved) | |
|---|---|---|---|
| Left stick | drive + slide | B | intake in/out backward? swap |
| Right stick | spin | D-pad left/right | intake slower/faster |
| D-pad | slow nudge | D-pad up/down | how far the brake goes down |
| LT / LB | intake in / out | X | flywheel backward? flip |
| RT (hold) | flywheel; buzz = ready | | |
| Y / A | flywheel faster / slower | | |
| RB (hold) | brake down, wheels stop | | |
| X / B (hold) | rollers in / out | | |

Any FTC-legal controller works; the screen shows that controller's own button names (`Pad.java`).

## The parts (one file each)

| File | Part of the robot |
|---|---|
| `DriveWheels.java` | The four drive motors, their directions, and the mecanum math (`mix`). |
| `Intake.java` | Intake motors. |
| `Rollers.java` | The two roller servos. |
| `Flywheel.java` | Shooter flywheel, held at a speed by its encoder. |
| `Brake.java` | Brake servo. |
| `HeadingSensor.java` | Pinpoint (or hub gyro): which way the robot faces, pod counts. |
| `SelfCheck.java` | Remembers which way the gyro and pods count (set by Wheel Test). |
| `Guard.java` | Safety wrapper: an unplugged mechanism switches itself off instead of freezing the drive. |
| `Pad.java` | Button names for the controller in use. |

Two rules the code follows everywhere:

1. **Nothing unplugged can stop the drive.** Every mechanism talks to its hub through a `Guard`. If the hub stops
   answering, that mechanism turns off, the screen says why, and it tries again when the driver uses it.
2. **Settings are changed on the robot, not in code.** Directions, speeds and distances are set with the controller
   and saved in small text files on the Control Hub (`/sdcard/FIRST/biobuzz_*.txt`), so they survive a reinstall.

## Putting the code on the robot

Android Studio: plug the Control Hub in by USB-C, pick the TeamCode module, press Run.
