# Robot Control Communication Protocol

## 1. Movement Control
- **Command Format**: `CMD_MOVE#<mode>#<x>#<y>#<speed>#<angle>\n`
- **Parameter Description**:
  - Mode: `"1"` or `"2"`, 1 for motion mode, 2 for gait mode
  - x, y: Step length, range `-35` to `35`
  - speed: Speed, range `2` to `10`
  - angle: Angle, range `-10` to `10` (default is `0`, straight walking)

## 2. LED Control
### LED Mode Setting
- **Command Format**: `CMD_LED_MOD#<mode>\n`
- **Mode Options**:
  - `"0"`: Off
  - `"1"`: Specified color
  - `"2"`: Chase mode
  - `"3"`: Blink mode
  - `"4"`: Breathing mode
  - `"5"`: Rainbow breathing mode

### LED Color Setting (Valid in modes 1 and 3)
- **Command Format**: `CMD_LED#<R>#<G>#<B>\n`
- **Parameter Description**: R, G, B represent red, green, and blue values, range 0-255

## 3. Ultrasonic Distance Measurement
- **Command Format**: `CMD_SONIC\n`
- **Return Value**: Distance to obstacle ahead (unit: centimeters)

## 4. Port Configuration
- **Video Port**: `8002`
- **Command Port**: `5002`

## 5. Buzzer Control
- **Turn On Buzzer**: `CMD_BUZZER#1\n`
- **Turn Off Buzzer**: `CMD_BUZZER#0\n`

## 6. Head Control
- **Horizontal Rotation**: `CMD_HEAD#0#<angle>\n`
- **Vertical Rotation**: `CMD_HEAD#1#<angle>\n`
- **Parameter Description**: angle range is `-90` to `90` degrees

## 7. Balance Function Control
- **Enable Balance Function**: `CMD_BALANCE#1\n`
- **Disable Balance Function**: `CMD_BALANCE#0\n`

## 8. Attitude Angle Control
- **Command Format**: `CMD_ATTITUDE#<roll>#<pitch>#<yaw>\n`
- **Parameter Range**: All angle values range from `-15` to `15` degrees

## 9. Position Control
- **Command Format**: `CMD_POSITION#<x>#<y>#<z>\n`
- **Parameter Range**:
  - x, y: `-40` to `40`
  - z: `-20` to `20`

## 10. Servo Power-off Control
- **Command Format**: `CMD_RELAX\n`
- **Parameter Description**: Each time the command is received, it toggles the servo enable/disable state. This command is not recommended; please use `CMD_SERVOPOWER` instead.

## 11. Power Voltage Query
- **Send Command**: `CMD_POWER\n`
- **Return Format**: `CMD_POWER#<load battery voltage>#<Raspberry Pi battery voltage>\n`

## 12. Camera View Movement
- **Command Format**: `CMD_CAMERA#<x>#<y>\n`
- **Parameter Description**: 
  - x: Left-right direction, range `-90` to `90`
  - y: Up-down direction, range `-90` to `90`

## 13. Servo Power Control
- **Power Off**: `CMD_SERVOPOWER#0\n`
- **Power On**: `CMD_SERVOPOWER#1\n`