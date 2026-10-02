import struct
import subprocess
import time
import argparse
# sudo ip link set can0 type can bitrate 500000
# sudo ip link set up can0

"""
basic test script for running motor commands from CAN

# Drill
0x02800100
    DRILL_SPEED:
    0x02800100
    DRILL_CURRENT_LIM
    0x02800110
    
# Shaft
0x02800200
    SHAFT_SPEED (RPM):
    0x02800200
    
# Centrifuge
02800300#0000
    CENTRIFUGE_DUTY:
    0x02800300
    
"""
DRILL_ADDR = 0x02800100
DRILL_SPEED_PARAM = DRILL_ADDR
DRILL_CURRENT_LIM = 0x02800110

SHAFT_ADDR = 0x02800200
SHAFT_RPM = SHAFT_ADDR

CENTRIFUGE_ADDR = 0x02800300
CENTRIFUGE_DUTY = CENTRIFUGE_ADDR


def sign(input):
    if input < 0:
        return -1
    elif input > 0:
        return 1
    else:
        return 0


def send_int16(interface, can_id, value):
    if not -32768 <= value <= 32767:
        raise ValueError("Value must fit in int16_t")

    data = struct.pack("<h", value).hex().upper()
    frame = f"{can_id:08X}#{data}"

    print(f"TX: {frame}")

    subprocess.run(
        ["cansend", interface, frame],
        check=True,
    )


# TODO: Set current limit
# Drill
# 02800100#0000
def test_drill_sweep():

    val = 0

    while 1:
        while val <= 32700:
            send_int16("can0", DRILL_SPEED_PARAM, val)
            val += 10
            time.sleep(0.001)

        while val >= -32700:
            send_int16("can0", DRILL_SPEED_PARAM, val)
            val -= 10
            time.sleep(0.001)


def test_drill(speed):

    send_int16("can0", DRILL_SPEED_PARAM, speed)


# Shaft
# 02800200#0000
# TODO: Shaft rotation to linear displacement:
#


def rpm_to_int16_t(rpm_in, rpm_min, rpm_max, out_min, out_max):
    return int((rpm_in - rpm_min) * (out_max - out_min) / (rpm_max - rpm_min) + out_min)


def test_shaft(rpm_in):

    if abs(rpm_in) == 0:
        send_int16("can0", SHAFT_RPM, 0)

    elif abs(rpm_in) < 5:
        print("LEDC cannot handle |RPM| < 5.")
        send_int16(
            "can0",
            SHAFT_RPM,
            rpm_to_int16_t(5 * sign(rpm_in), MIN_RPM, MAX_RPM, -32768, 32768),
        )

    else:
        send_int16(
            "can0", SHAFT_RPM, rpm_to_int16_t(rpm_in, MIN_RPM, MAX_RPM, -32768, 32768)
        )


def run_shaft_test(position):
    direction = 0

    if position == 0:
        direction = 1
    elif position == 1:
        direction = -1
    else:
        print("Invalid test input.")

        send_int16(
            "can0", SHAFT_RPM, rpm_to_int16_t(0, MIN_RPM, MAX_RPM, -32768, 32768)
        )
        return

    try:
        send_int16(
            "can0",
            SHAFT_RPM,
            rpm_to_int16_t(25 * direction, MIN_RPM, MAX_RPM, -32768, 32768),
        )
        time.sleep(6)

        send_int16(
            "can0",
            SHAFT_RPM,
            rpm_to_int16_t(75 * direction, MIN_RPM, MAX_RPM, -32768, 32768),
        )
        time.sleep(8)

        send_int16(
            "can0",
            SHAFT_RPM,
            rpm_to_int16_t(25 * direction, MIN_RPM, MAX_RPM, -32768, 32768),
        )
        time.sleep(5)

    finally:
        print("\nStopping shaft...")

        send_int16(
            "can0", SHAFT_RPM, rpm_to_int16_t(0, MIN_RPM, MAX_RPM, -32768, 32768)
        )


MIN_RPM = -180
MAX_RPM = 180


# Centrifuge
# 02800300#00
# 00
test_cont = 0


def test_centrifgue(duty):

    if duty == 0.0:
        send_int16("can0", CENTRIFUGE_DUTY, 0)

    if abs(duty) < 0.05:
        print("Centrifuge duty must have a greater magnitude than 0.05")
        return
    duty_int = int(duty * 32767)

    # ramp up, down, and to zero
    if test_cont == 1:
        send_int16("can0", CENTRIFUGE_DUTY, duty_int)
        time.sleep(5 * (duty * 2))
        send_int16("can0", CENTRIFUGE_DUTY, -duty_int)
        time.sleep(5 * (duty * 2))
        send_int16("can0", CENTRIFUGE_DUTY, 0)

    else:
        send_int16("can0", CENTRIFUGE_DUTY, duty_int)


def main():
    parser = argparse.ArgumentParser(
        description="Space Resources CAN hardware test utility"
    )

    subparsers = parser.add_subparsers(
        dest="test",
        required=True,
        help="Test to run",
    )

    # Drill
    drill_parser = subparsers.add_parser(
        "drill",
        help="Run continuous drill speed sweep",
    )

    drill_parser.add_argument(
        "drill",
        type=int,
        help="Send drill speed",
    )

    # Shaft
    shaft_parser = subparsers.add_parser(
        "shaft",
        help="Set shaft RPM",
    )

    shaft_parser.add_argument(
        "RPM",
        type=int,
        help=f"Signed int16 shaft command ({MIN_RPM} to {MAX_RPM})",
    )

    shaft_test = subparsers.add_parser("shaft_test", help="Start shaft test")

    shaft_test.add_argument(
        "POS",
        type=int,
        help="Start shaft test",
    )

    # Centrifuge
    centrifuge_parser = subparsers.add_parser(
        "centrifuge",
        help="Set centrifuge speed",
    )
    centrifuge_parser.add_argument(
        "speed",
        type=float,
        help="Signed int16 speed command (-32768 to 32767)",
    )

    args = parser.parse_args()

    if args.test == "drill":
        test_drill(args.drill)

    elif args.test == "shaft":
        test_shaft(args.RPM)

    elif args.test == "shaft_test":
        run_shaft_test(args.POS)

    elif args.test == "centrifuge":
        if abs(args.speed) <= 1.0:
            test_centrifgue(args.speed)
        else:
            print("Ensure float is within -1<in<1!")


if __name__ == "__main__":
    main()
