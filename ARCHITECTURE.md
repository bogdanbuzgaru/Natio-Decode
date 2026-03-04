# FTC Team 19054 NeuroBotix - TeamCode Comprehensive Documentation

This document provides detailed coverage of all classes, mathematical logic, control algorithms, and architecture within the `teamcode` package for FTC Team 19054 NeuroBotix. It is aligned with the FTC Control Award guidelines and includes mermaid diagrams for every subsystem, math model, and autonomous flow.

---

## Table of Contents

1. [Overall System Architecture](#1-overall-system-architecture)
2. [TeleOp Control Loop](#2-teleop-control-loop)
3. [Autonomous State Machine](#3-autonomous-state-machine)
4. [Path Planning Routes](#4-path-planning-routes)
5. [Mecanum Drive Math](#5-mecanum-drive-math)
6. [Shooter Ballistics Pipeline](#6-shooter-ballistics-pipeline)
7. [Position and Zone Detection](#7-position-and-zone-detection)
8. [LinearEquation and Intersection Math](#8-linearequation-and-intersection-math)
9. [Color Sensor Logic](#9-color-sensor-logic)
10. [Turret Control Loop](#10-turret-control-loop)
11. [Shooter Subsystem Control](#11-shooter-subsystem-control)
12. [Pose Persistence Sequence](#12-pose-persistence-sequence)

---

## 1. Overall System Architecture

```mermaid
flowchart TD
    subgraph OpModes["OpModes Layer"]
        TeleOp["telop.java"]
        Testing["Testing.java"]
        AutoRF["RedFar"]
        AutoRC["RedClose"]
        AutoBF["BlueFar"]
        AutoBC["BlueClose"]
    end
    subgraph Subsystems["Subsystems Package"]
        Shooter["Shooter\nflywheel1 flywheel2\nhood barrier"]
        Index["Index\nindexMotor"]
        Intake["Intake\nintakeMotor"]
        Turret["Turret\nturretServo1/2/3"]
    end
    subgraph Math["Math Package"]
        Movement["Movement\nmecanum drive"]
        ShooterCalc["ShooterCalculations\nballistics"]
        Position["Position\nfield geometry"]
        Sensor["Sensor\ncolor HSV"]
        LinearEq["LinearEquation\nline segments"]
    end
    subgraph Pathing["Autonomous / Pathing"]
        Follower["PedroPathing Follower"]
        PathChain["PathChain\nBezierLine / BezierCurve"]
        StateMachine["StateMachine\nSHOOT/DETECT/COLLECT"]
        Pinpoint["GoBildaPinpointDriver\nodometry"]
    end
    Testing --> Shooter
    Testing --> Index
    Testing --> Intake
    Testing --> Turret
    Testing --> Movement
    Testing --> Pinpoint
    Testing --> Position
    Testing --> ShooterCalc
    AutoRF --> Follower
    AutoRC --> Follower
    AutoBF --> Follower
    AutoBC --> Follower
    Follower --> PathChain
    StateMachine --> Follower
    Position --> LinearEq
    Position --> ShooterCalc
    Sensor --> Testing
    Pinpoint --> Position
```

---

## 2. TeleOp Control Loop

```mermaid
flowchart TD
    A([OpMode Start]) --> B[Init Hardware\npinpoint movement shooter\nindex intake turret pos]
    B --> C[waitForStart]
    C --> D{isStopRequested?}
    D -- YES --> Z([Stop])
    D -- NO --> E[loop iteration]
    E --> F[pinpoint.update]
    F --> G[pos.update pose]
    G --> H[movement.movementLoop gamepad1]
    H --> I[intake.take gamepad1]
    I --> J[index.feed gamepad1]
    J --> K[pos.chooseAlliance gamepad2]
    K --> L[calculateShootingParameters]
    L --> M[turret.setHeading and setTargetAngle]
    M --> N{pos.activateOrientation?}
    N -- YES --> O[turret.update]
    N -- NO --> P[turret.goNeutral]
    O --> Q{pos.shootClose or shootHigh?}
    P --> Q
    Q -- shootClose --> R[shooter ticks close\nindex.autoFeed\nshooter.raiseBarrier]
    Q -- shootHigh --> S[shooter ticks high\nindex.autoFeed\nshooter.raiseBarrier]
    Q -- neither --> T[shooter manual\nindex.feed manual]
    R --> D
    S --> D
    T --> D
```

---

## 3. Autonomous State Machine

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> SHOOT : loop begins
    SHOOT --> DETECT : ring shot or timeout
    DETECT --> COLLECT : game element found
    COLLECT --> SHOOT : ring collected
    DETECT --> SHOOT : element not found
    SHOOT --> STOP : opMode stop called
    COLLECT --> STOP : opMode stop called
    DETECT --> STOP : opMode stop called
    STOP --> [*] : writes FinalPos.txt x y heading
```

---

## 4. Path Planning Routes

```mermaid
flowchart LR
    subgraph RF["RedFar"]
        RF1["Start\n87, 8"] -->|BezierLine| RF2["Collect\n134, 9"]
        RF2 -->|BezierLine| RF3["Return\n94, 8.5"]
        RF3 -->|BezierLine| RF4["Park\n111, 16"]
    end
    subgraph BF["BlueFar"]
        BF1["Start\n57, 8"] -->|BezierLine| BF2["Collect\n10, 9"]
        BF2 -->|BezierLine| BF3["Return\n50, 8.5"]
    end
    subgraph RC["RedClose"]
        RC1["Start\n120, 144"] -->|BezierCurve 3pt| RC2["Shoot Zone\n84, 84"]
        RC2 -->|BezierCurve 4pt| RC3["Score\n127, 60"]
        RC3 -->|BezierCurve| RC4["Return\n84, 84"]
        RC4 -->|BezierCurve 43deg| RC5["Score 2\n129, 63"]
    end
    subgraph BC["BlueClose"]
        BC1["Start\n24, 144"] -->|BezierCurve 3pt| BC2["Shoot Zone\n60, 84"]
        BC2 -->|BezierCurve 4pt| BC3["Score\n17, 60"]
        BC3 -->|BezierCurve| BC4["Return\n60, 84"]
        BC4 -->|BezierCurve 137deg| BC5["Score 2\n15, 63"]
    end
```

---

## 5. Mecanum Drive Math

```mermaid
flowchart TD
    GP["Gamepad Input"] --> LX["left_stick_x = x\nx times 1.1 strafing correction"]
    GP --> LY["left_stick_y = y\nnegated"]
    GP --> RX["right_stick_x = rx\nrotation"]
    LX --> CALC["Motor Power Equations\nFL = y + x + rx\nBL = y - x + rx\nFR = y - x - rx\nBR = y + x - rx"]
    LY --> CALC
    RX --> CALC
    CALC --> FL["frontLeft.setPower"]
    CALC --> BL["backLeft.setPower"]
    CALC --> FR["frontRight.setPower"]
    CALC --> BR["backRight.setPower"]
```

---

## 6. Shooter Ballistics Pipeline

```mermaid
flowchart TD
    IN["Inputs\nrobotX robotY\nrobotVelocityX robotVelocityY\ngoalX=130 goalY=130\ngoalHeight=45in\nentryAngle=-45deg"]
    IN --> D1["Distance and Angle\ndx = goalX - robotX\ndy = goalY - robotY\ndistance = sqrt dx2 plus dy2\nangleToGoal = atan2 dy dx"]
    D1 --> D2["Launch Angle Alpha\nalpha = atan 2h divided d minus tan entryAngle\ncosAlpha and tanAlpha computed"]
    D2 --> D3["Initial Speed v0\nnumerator = g times d squared\ndenom = 2 cos2Alpha times d tanAlpha minus h\nv0 = sqrt num divided denom\ng = 386.1 in per s2"]
    D3 --> D4["Velocity Compensation\nrobotSpeed = sqrt vx2 plus vy2\nrobotVelAngle = atan2 vy vx\ndeltaAngle = velAngle minus angleToGoal\nvRadial = neg cos delta times speed\nvTangential = sin delta times speed"]
    D4 --> D5["Compensated Angle\nvx = v0 cosAlpha\ntimeToGoal = d divided vx\nvxComp = vx plus vRadial\nvxNew = sqrt vxComp2 plus vTang2\nlaunchAngle = atan vy divided vxNew\nclamped 35deg to 45deg"]
    D5 --> D6["Output Parameters\nhoodServoPos = linear interp angle\nflywheelSpeed = a times launchSpeed plus b\nturretOffset = atan vTang divided vxComp\ntargetTurret = angleToGoal plus offset"]
    D6 --> OUT["ShootingParameters\nlaunchAngle rad\nlaunchSpeed in per s\nhoodServoPosition\nflywheelSpeed ticks\ndistanceToGoal\nturretOffsetAngle\ntargetTurretAngle"]
```

---

## 7. Position and Zone Detection

```mermaid
flowchart TD
    POSE["Pose2D from Pinpoint\nx y heading"] --> UPD["Position.update pose\nRecalculate heading and signs"]
    UPD --> DIST["Bounding Box\nl=12.87in L=14.49in\ndistY = l cosTheta plus L sinTheta\ndistX = L cosTheta plus l sinTheta\nMaxX and MaxY = center plus dist divided 2"]
    DIST --> OFF["Offsets\noffsetAbscissa = acos deltaX divided semiDiag\noffsetOrdinate = acos deltaY divided semiDiag\nsemiDiagonal = 9.69in"]
    OFF --> LE["4 Corner LinearEquations\ntopLeft topRight\nbottomLeft bottomRight\nusing semiDiag times sin offset"]
    LE --> ZD["Zone Detection"]
    ZD --> BIG["isCenterInBigTriangle\ny >= 72\nx >= 144 minus y\nx <= y\nresult shootClose"]
    ZD --> SMALL["isCenterInSmallTriangle\ny <= 24\nx >= 48 plus y\nx <= 96 minus y\nresult shootHigh"]
    ZD --> TANG["isTangentToTriangle\n4 robot edges times 2 triangle lines\nareIntersecting check\nresult shootClose or shootHigh"]
    ZD --> ORI["activateOrientation\nhypoHigh = dist center to 130,144 <= 90\nhypoLow = dist center to 72,0 <= 50\nresult turret tracking ON"]
```

---

## 8. LinearEquation and Intersection Math

```mermaid
flowchart TD
    CTOR["LinearEquation x1 y1 x2 y2"] --> CALC["calculate\nxCoeff = y1 minus y2\nyCoeff = x2 minus x1\nconstant = x1 times y2 minus x2 times y1\nslope = neg xCoeff divided yCoeff"]
    CALC --> LINE["Line form\nxCoeff times x plus yCoeff times y plus constant = 0"]
    subgraph FieldLines["Fixed Field Boundaries"]
        LBT["leftBigTriangle\n14,130 to 72,72"]
        RBT["rightBigTriangle\n130,130 to 72,72"]
        LST["leftSmallTriangle\n48,0 to 72,24"]
        RST["rightSmallTriangle\n72,24 to 96,0"]
    end
    subgraph RobotLines["Dynamic Robot Edges"]
        TL["topLeft"]
        TR["topRight"]
        BL["bottomLeft"]
        BR["bottomRight"]
    end
    FieldLines --> INTER["areIntersecting L1 L2\nif slope diff < 1e-9 then parallel false\nx = b gamma minus c beta divided a beta minus b alpha\ny = c alpha minus a gamma divided a beta minus b alpha\ncheck x and y within both segment ranges"]
    RobotLines --> INTER
    INTER --> RESULT["true or false\nrobot edge overlaps shooting zone"]
```

---

## 9. Color Sensor Logic

```mermaid
flowchart TD
    HW["NormalizedColorSensor\nGAIN = 25.0"] --> UPD["updateColors\nnormalizedColors toColor\nColor.colorToHSV to hsvValues array"]
    UPD --> HSV["HSV Values\nhsvValues 0 = Hue 0 to 360\nhsvValues 1 = Saturation 0 to 1\nhsvValues 2 = Value 0 to 1"]
    HSV --> PURPLE{"isPurple\nhue 210 to 330\nsaturation > 0.4\nvalue > 0.1"}
    HSV --> GREEN{"isGreen\nhue 90 to 180\nsaturation > 0.4\nvalue > 0.1"}
    PURPLE -- YES --> PA["Purple element detected\nARTIFACT_PURPLE"]
    PURPLE -- NO --> PN["Not purple"]
    GREEN -- YES --> GA["Green element detected\nARTIFACT_GREEN"]
    GREEN -- NO --> GN["Not green"]
    PA --> AUTON["Autonomous DETECT state\nbranch to collection path"]
    GA --> AUTON
```

---

## 10. Turret Control Loop

```mermaid
flowchart TD
    INPUT["Inputs\nheading from Pinpoint\ntargetAngle from Position\noffsetAngle from ShooterCalc"]
    INPUT --> ERR["error = heading minus angle\nif error > 180 subtract 360\nif error < -180 add 360"]
    ERR --> DIR{"Direction?"}
    DIR -- "error < 0 turnLeft" --> LEFT["position = 0.5 plus 0.5 times abs targetAngle div 165.5\nservo1 servo2 servo3 = position"]
    DIR -- "error > 0 turnRight" --> RIGHT["position = 0.5 minus 0.5 times abs targetAngle div 165.5\nservo1 servo2 servo3 = position"]
    DIR -- "error = 0" --> NEUT["goNeutral\nservo1 servo2 servo3 = 0.5"]
    INPUT --> WRAP["setTargetAngle wraparound\n> 180 subtract 360\n< -180 add 360\nanti-flip at boundary\nif sign change and 180 minus abs angle < 8deg\ncompensate continuously"]
    LEFT --> SERVO["turretServo1 turretServo2 turretServo3\n3 servos mirrored"]
    RIGHT --> SERVO
    NEUT --> SERVO
```

---

## 11. Shooter Subsystem Control

```mermaid
flowchart TD
    INIT["Shooter init\nflywheel1 FORWARD\nflywheel2 REVERSE\nRUN_USING_ENCODER\nlowerBarrier to 0.6"]
    subgraph MANUAL["Manual Speed Presets gamepad"]
        D1["dPad Left = 1456 ticks per s"]
        D2["Circle = 1619 ticks per s"]
        D3["dPad Right = 2300 ticks per s"]
        D4["Cross = 1280 ticks per s"]
    end
    subgraph UPDATE["update loop"]
        U1["flywheel1.setVelocity ticks"]
        U2["flywheel2.setVelocity ticks"]
        U3["adaptiveHood"]
        U4["hood.setPosition targetHood"]
    end
    INIT --> MANUAL
    MANUAL --> UPDATE
    U3 --> ADAPTIVE["adaptiveHood\nerror = motor1.getVelocity minus ticks\ntargetHood = min ticks div 2400 plus min hoodPos plus error times 0.0005 and 1 and 1"]
    subgraph BARRIER["Barrier Control"]
        LB["lowerBarrier servo = 0.6 safe default"]
        RB["raiseBarrier servo = 0.4 ready to fire"]
    end
    UPDATE --> BARRIER
    NOTE["setTicks guard\nonly update if delta ticks >= 50\nprevents micro-oscillation"]
```

---

## 12. Pose Persistence Sequence

```mermaid
sequenceDiagram
    participant Auto as Autonomous OpMode
    participant Follower as PedroPathing Follower
    participant Sensor as Sensor Color
    participant FSM as StateMachine
    participant File as FinalPos.txt
    Auto->>Follower: init set starting pose
    loop Autonomous loop
        Auto->>Follower: follower.update
        Auto->>FSM: fsm.update state
        FSM-->>Auto: current state SHOOT DETECT COLLECT
        alt SHOOT state
            Auto->>Follower: followPath Path1
        else DETECT state
            Auto->>Sensor: updateColors
            Sensor-->>Auto: isPurple or isGreen
        else COLLECT state
            Auto->>Follower: followPath Path2
        end
    end
    Auto->>Follower: stop called getPose x y heading
    Auto->>File: WriteFile FinalPos.txt x newline y newline heading
    Note over File: Pose handed off to TeleOp
    participant TeleOp as TeleOp OpMode
    TeleOp->>File: ReadFile FinalPos.txt
    TeleOp->>TeleOp: Restore pose for field-relative control
```

---

## Mathematical Formulas Reference

### Mecanum Drive

```
FL = y + x + rx
BL = y - x + rx
FR = y - x - rx
BR = y + x - rx
```

- `x` = strafe (left_stick_x * 1.1)
- `y` = forward (-left_stick_y)
- `rx` = rotation (right_stick_x)

### Projectile Launch Speed

```
alpha = atan(2h/d - tan(entryAngle))
v0 = sqrt( g * d^2 / (2 * cos^2(alpha) * (d*tan(alpha) - h)) )
g = 386.1 in/s^2
```

### Flywheel Speed (Linear Calibration)

```
flywheelSpeed = a * launchSpeed + b
```

### Hood Servo Position

```
slope = (minServoPos - maxServoPos) / (minHoodAngle - maxHoodAngle)
position = slope * (angleDeg - minHoodAngle) + minServoPos
position = clamp(position, 0.0, 1.0)
```

### Turret Velocity Offset

```
vRadial = -cos(velAngle - goalAngle) * robotSpeed
vTangential = sin(velAngle - goalAngle) * robotSpeed
turretOffset = atan(vTangential / vxCompensated)
targetTurret = angleToGoal + turretOffset
```

### Color Sensing HSV Thresholds

```
Purple: hue in [210, 330], saturation > 0.4, value > 0.1
Green:  hue in [90,  180], saturation > 0.4, value > 0.1
```

### Robot Bounding Box

```
distY = l*cos(heading) + L*sin(heading)
distX = L*cos(heading) + l*sin(heading)
l = 12.874 in,  L = 14.488 in,  semiDiagonal = 9.691 in
```

### Shooting Zone (Big Triangle - shootClose)

```
y >= 72
x >= 144 - y
x <= y
```

### Shooting Zone (Small Triangle - shootHigh)

```
y <= 24
x >= 48 + y
x <= 96 - y
```

---

## FTC Control Award Checklist

| Criteria | Implementation |
|----------|----------------|
| Robust architecture | Modular class-based design, isolated subsystems |
| Clear control logic | Encapsulated motor/servo logic, teleop/autonomous split |
| Sensor integration | Full color sensor with HSV thresholds |
| Error handling | isStopRequested checks, adaptive hood control |
| Mathematical modeling | Ballistics, geometry, kinematics classes |
| Data logging | Telemetry and pose data via FinalPos.txt |
| Tuning support | Centralized Constants, path abstractions |
| Documentation | This file plus inline code comments |
| Best practices | Hardware map abstraction, safe initialization |
| Testing | Pre-movement safety checks, neutral resets |

---

## Software Engineering Best Practices

- **Separation of concerns:** Each subsystem has a dedicated class.
- **Centralized constants:** Tuning via `Constants` and config files.
- **Explicit initialization:** All hardware mapped and set to zero/neutral before start.
- **Robust error handling:** Defensive programming with isStopRequested and adaptive controls.
- **Safe gamepad integration:** No uncontrolled actions; all powers managed via state.
- **Reusability:** Autonomous uses path chains and state machines for flexible routine design.
- **Pose persistence:** Final robot pose written to file for cross-mode coordinate continuity.

---

## References

- [TeamCode Source](https://github.com/bogdanbuzgaru/Natio-Decode/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode)
- [OpModes](https://github.com/bogdanbuzgaru/Natio-Decode/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opModes)
- [Autonomous Classes](https://github.com/bogdanbuzgaru/Natio-Decode/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous)
- [Subsystems](https://github.com/bogdanbuzgaru/Natio-Decode/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems)
- [Math Helpers](https://github.com/bogdanbuzgaru/Natio-Decode/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/math)
