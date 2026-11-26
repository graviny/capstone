from gpiozero import DigitalInputDevice
import time
import move
import RPIservo
import robotLight
import functions


IR_GPIO = 12  # GPIO pin
TOLERANCE = 0.5  # tolerance for NEC decoding
MIN_NORMAL_PULSES = 20  # minimum pulses for valid NEC code
REPEAT_PULSES = 25

repeat_code = "repeat"
# Robot movement parameters
speed_set = 60
direction_command = 'no'
turn_command = 'no'
# Store pulse data: (signal_level, timestamp_microsec)
pulse_timestamps = []

# Key mapping 
key_map = {
    0xA2: "1",
    0x62: "2",
    0xE2: "3",
    0x22: "4",
    0x02: "5",
    0xC2: "6",
    0xE0: "7",
    0xA8: "8",
    0x90: "9",
    0x98: "0",
    0x68: "*",
    0xB0: "#",
    0x18: "UP",
    0x4A: "DOWN",
    0x10: "LEFT",
    0x5A: "RIGHT",
    0x38: "OK",
}
# --------------------------------------------------------------------------------------------------


def pulse_callback(level):
    timestamp = time.time_ns() // 1000
    pulse_timestamps.append((level, timestamp))
    #print(f"[RAW] Level: {level}, Timestamp: {timestamp}μs | Pulses captured: {len(pulse_timestamps)}")
    if len(pulse_timestamps) > 200:
        pulse_timestamps.pop(0)


def calculate_time_diff(t1, t2):
    if t1 > t2:
        return (0xFFFFFFFF - t1) + t2
    return t2 - t1


def is_within_tolerance(measured, expected):
    min_val = expected * (1 - TOLERANCE)
    max_val = expected * (1 + TOLERANCE)
    return min_val <= measured <= max_val


def decode_nec_debug():
    """
    Returns (address, data) if decoding succeeds; None otherwise.
    """
    global pulse_timestamps
    # Skip if not enough pulses 
    if len(pulse_timestamps) < 4:
        return None

    # Copy and clear pulse buffer
    pulses = pulse_timestamps.copy()
    pulse_timestamps = []
    pulse_count = len(pulses)
    #print(f"[DEBUG] Processing {pulse_count} pulses")

    # Skip NEC repeat codes
    if pulse_count == 4:
        return None

    # Skip if pulse count is too low
    if pulse_count < MIN_NORMAL_PULSES:
        return None
    
    if pulse_count <= REPEAT_PULSES:
        return repeat_code

    # Verify NEC header (9ms high + 4.5ms low)
    t0, t1, t2 = [p[1] for p in pulses[:3]] 
    hdr_mark = calculate_time_diff(t0, t1)
    hdr_space = calculate_time_diff(t1, t2)
    if not (is_within_tolerance(hdr_mark, 9000) and is_within_tolerance(hdr_space, 4500)):
        return None

    # Decode 32-bit NEC data
    data_bits = []
    valid_bits = 0
    for i in range(2, pulse_count - 2, 2):
        # Extract timestamps for current bit (high + low)
        t_prev = pulses[i][1]
        t_curr = pulses[i+1][1]
        t_next = pulses[i+2][1]

        # Check bit high time (must be ~560μs)
        bit_mark = calculate_time_diff(t_prev, t_curr)
        if not is_within_tolerance(bit_mark, 560):
            continue

        # Determine bit value (0 = ~560μs low, 1 = ~1690μs low)
        bit_space = calculate_time_diff(t_curr, t_next)
        if is_within_tolerance(bit_space, 1690):
            data_bits.append(1)
            valid_bits += 1
        elif is_within_tolerance(bit_space, 560):
            data_bits.append(0)
            valid_bits += 1

    # Skip if incomplete data (less than 32 valid bits)
    if valid_bits < 32:
        return None

    # Convert bits to address/data 
    address = int(''.join(map(str, data_bits[0:8])), 2)
    address_inv = int(''.join(map(str, data_bits[8:16])), 2)
    data = int(''.join(map(str, data_bits[16:24])), 2)
    data_inv = int(''.join(map(str, data_bits[24:32])), 2)

    # Verify data integrity (XOR check)
    if (address ^ address_inv) == 0xFF and (data ^ data_inv) == 0xFF:
        return (address, data)
    else:
        return None


def robotCtrl(command):
    global direction_command, turn_command, scGear, RL, fuc
    if 'UP' == command:
        direction_command = 'forward'
        scGear.moveAngle(0, 0)
        move.move(speed_set, 1, "mid")
        RL.both_on(0, 255, 0)
        print("forward")
    
    elif 'DOWN' == command:
        direction_command = 'backward'
        scGear.moveAngle(0, 0)
        move.move(speed_set, -1, "mid")
        RL.both_on(255, 0, 0)
        print("backward")

    elif 'DS' in command:
        direction_command = 'no'
        scGear.moveAngle(0, 0)
        move.motorStop()
        if turn_command == 'left':
            RL.RGB_left_on(0, 255, 0)
        elif turn_command == 'right':
            RL.RGB_right_on(0, 255, 0)
        elif turn_command == 'no':
            RL.both_off()

    elif 'LEFT' == command:
        turn_command = 'left'
        scGear.moveAngle(0, 30)
        move.move(speed_set, 1, "mid")
        RL.RGB_left_on(0, 255, 0)
        print("turn_left")
        time.sleep(0.15)
        

    elif 'RIGHT' == command:
        turn_command = 'right'
        scGear.moveAngle(0, -30)
        move.move(speed_set, 1, "mid")
        RL.RGB_right_on(0, 255, 0)
        print("turn_right")
        time.sleep(0.15)

    elif 'TS' in command:
        turn_command = 'no'
        scGear.moveAngle(0, 0)
        move.motorStop()
        if direction_command == 'forward':
            RL.both_on(0, 255, 0) 
        elif direction_command == 'backward':
            RL.both_on(255, 0, 0)     
        elif direction_command == 'no':
            RL.both_off()

    elif '1' == command:
        scGear.singleServo(1, 1, 7)
        print("head_left")

    elif '3' == command:
        scGear.singleServo(1, -1, 7)
        print("head_right")
        
    elif '2' == command:
        scGear.singleServo(2, 1, 7)
        print("head_up")

    elif '5' == command:
        scGear.singleServo(2, -1, 7)
        print("head_down")

    elif 'HSTOP' in command:
        scGear.stopWiggle()

    elif '*' == command:
        scGear.moveAngle(0, 0)
        scGear.moveAngle(1, 0)
        scGear.moveAngle(2, 0)
        fuc.keepDistance()
        print("keep distance")

    elif '#' == command:
        scGear.moveAngle(0, 0)
        scGear.moveAngle(1, 0)
        scGear.moveAngle(2, 0)
        fuc.automatic()
        print("automatic")

    elif 'OK' == command:    
        fuc.pause()
        scGear.moveAngle(0, 0)
        scGear.moveAngle(1, 0)
        scGear.moveAngle(2, 0)
        move.motorStop()
        print("stop")
            

def main(): 
    global scGear, RL, fuc

    # GPIOZero IR Receiver Initialization
    ir_receiver = DigitalInputDevice(
        pin=IR_GPIO,
        pull_up=True,  
        bounce_time=0.0003  # 300μs debounce (filters noise)
    )
    ir_receiver.when_activated = lambda: pulse_callback(1)
    ir_receiver.when_deactivated = lambda: pulse_callback(0)

    scGear = RPIservo.ServoCtrl()
    scGear.moveInit()
    scGear.start()

    RL = robotLight.RobotLight() 

    fuc = functions.Functions()
    fuc.setup()
    fuc.start()

    print("IR Robot Controller Started")
    print("Press Ctrl+C to exit...")
    button_command = "OK"
    try:
        while True:
            time.sleep(0.4)
            result = decode_nec_debug()
            if result is not None:
                if repeat_code not in result:
                    address, data = result
                    button_command = key_map.get(data, f"unknown key(0x{data:02X})")
                    print(f"command：{button_command} ")

                if button_command.endswith("UP") or button_command.endswith("DOWN"):
                    robotCtrl(button_command)
                    time.sleep(1)
                    robotCtrl("DS")
                elif button_command.endswith("LEFT") or button_command.endswith("RIGHT"):
                    robotCtrl(button_command)
                    time.sleep(1)
                    robotCtrl("TS")
                elif button_command.endswith("1") or button_command.endswith("3") or button_command.endswith("2") or button_command.endswith("5"):
                    robotCtrl(button_command)
                    time.sleep(0.5)
                    robotCtrl("HSTOP")
                else:
                    robotCtrl(button_command)
    except KeyboardInterrupt:
        print("\nexit")
    finally:
        ir_receiver.close()  

if __name__ == "__main__":
    main()