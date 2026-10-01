#include <Arduino.h>
#include <INA238.h>
#include <VescUart.h>
#include <driver/sdm.h>

#include <board_support.hpp>
#include <chrono>
#include <hi_can_twai.hpp>
#include <optional>
#include <thread>

using namespace bsp;
/*

TODO: THE FIRMWARE SHOULD BE DESIGNED SUCH THAT EACH GROUP
TODO: HAS ITS RESPECTIVE STATE MACHINE THAT HANDLES PARAMETERS.

| Desired value | `int16_t` hex | CAN payload |
| ------------: | ------------: | ----------: |
|    **-32768** |      `0x8000` |  **`0080`** |
|        -32767 |      `0x8001` |      `0180` |
|         -1000 |      `0xFC18` |      `18FC` |
|          -256 |      `0xFF00` |      `00FF` |
|          -129 |      `0xFF7F` |      `7FFF` |
|          -128 |      `0xFF80` |      `80FF` |
|            -1 |      `0xFFFF` |      `FFFF` |
|         **0** |      `0x0000` |  **`0000`** |
|             1 |      `0x0001` |      `0100` |
|           127 |      `0x007F` |      `7F00` |
|           128 |      `0x0080` |      `8000` |
|           255 |      `0x00FF` |      `FF00` |
|           256 |      `0x0100` |      `0001` |
|          1000 |      `0x03E8` |      `E803` |
|     **32767** |      `0x7FFF` |  **`FF7F`** |

*/

/*
  Testing config for DRV8874 Half Bridge Driver:
*/
static constexpr pin_pair_t DRILL_PINS{GPIO_NUM_15, GPIO_NUM_16};
static constexpr gpio_num_t DRILL_CURRENT_LIMIT = GPIO_NUM_40;
static constexpr gpio_num_t DRILL_CURRENT_SENSE = bsp::A1;
static constexpr gpio_num_t DRILL_FAULT = bsp::A2;
static constexpr gpio_num_t SLEEP = GPIO_NUM_39;

static constexpr uint8_t PWM_BITS = 12;
static constexpr uint16_t PWM_RES = 1 << PWM_BITS;
static constexpr uint16_t PWM_RES_HALF = (1 << (PWM_BITS - 1));
static constexpr uint32_t PWM_FREQ = 1500; // Hz

static constexpr uint32_t PWM_DEADBAND =
    2; // 2 PWM steps of enforced deadband to reset cycle-by-cycle current
       // chopping
static constexpr uint32_t PWM_MAX = (1 << PWM_BITS) - 1 - PWM_DEADBAND; // 4095

class MotorDriver {
public:
  enum class direction { FORWARD, STOPPED, BACKWARD };
  MotorDriver(const pin_pair_t &pins) : _pins(pins) {
    pinMode(pins.first, OUTPUT);
    pinMode(pins.second, OUTPUT);
  }

  virtual ~MotorDriver() {
    digitalWrite(_pins.first, LOW);
    digitalWrite(_pins.second, LOW);
    pinMode(_pins.first, INPUT);
    pinMode(_pins.second, INPUT);
  }

  void set_speed(int16_t speed) {
    speed = map(speed, std::numeric_limits<int16_t>::min(),
                std::numeric_limits<int16_t>::max(), -PWM_MAX, PWM_MAX);

    // printf(std::format("SET MOTOR SPEED: {}\n", speed).c_str());

    direction current_dir = direction::STOPPED;
    if (speed > 0)
      current_dir = direction::FORWARD;
    else if (speed < 0)
      current_dir = direction::BACKWARD;
    bool dir_changed = (current_dir != _prev_direction);
    _prev_direction = current_dir;

    if (dir_changed) {
      if (current_dir == direction::FORWARD) {
        pinMode(_pins.second, OUTPUT);
        digitalWrite(_pins.second, LOW);
        analogWrite(_pins.first, 1);
        analogWriteResolution(_pins.first, PWM_BITS);
        analogWriteFrequency(_pins.first, PWM_FREQ);
      } else if (current_dir == direction::BACKWARD) {
        pinMode(_pins.first, OUTPUT);
        digitalWrite(_pins.first, LOW);
        analogWrite(_pins.second, 1);
        analogWriteResolution(_pins.second, PWM_BITS);
        analogWriteFrequency(_pins.second, PWM_FREQ);
      } else {
        pinMode(_pins.first, OUTPUT);
        pinMode(_pins.second, OUTPUT);
        digitalWrite(_pins.first, LOW);
        digitalWrite(_pins.second, LOW);
      }
    }

    if (speed > 0)
      analogWrite(_pins.first, speed);
    else if (speed < 0)
      analogWrite(_pins.second, -speed);
  }

private:
  direction _prev_direction = direction::STOPPED;

  pin_pair_t _pins;
};

MotorDriver drill_driver{DRILL_PINS};

/*
  Test config DRV8824 Stepper Motor Driver (SHAFT):
*/

// STEP, DIR
static constexpr pin_pair_t SHAFT_PINS{GPIO_NUM_42, GPIO_NUM_41};

// MICROSTEP CONFIG
static const int M0_PIN = GPIO_NUM_40;
static const int M1_PIN = GPIO_NUM_39;
static const int M2_PIN = GPIO_NUM_38;

const uint8_t MS_TABLE[] = {0b000, 0b001, 0b010, 0b011, 0b100, 0b111};

static constexpr int MOTOR_RPM = 5;

constexpr uint32_t MIN_RPM = 5;
constexpr uint32_t MAX_RPM = 180;
uint32_t SET_STEP_FREQ = 0;

enum class microstep : uint8_t {
  FULL = 0,
  HALF,
  QUARTER,
  EIGHTH,
  SIXTEENTH,
  THIRTY_SECOND
};

void set_microstep(microstep mode) {
  const uint8_t ms_config = MS_TABLE[static_cast<uint8_t>(mode)];
  digitalWrite(M0_PIN, (ms_config >> 2) & 0x01);
  digitalWrite(M1_PIN, (ms_config >> 1) & 0x01);
  digitalWrite(M2_PIN, ms_config & 0x01);
}

// CONFIG FOR 1.8 DEG NEMA 23: 360/1.8 = 200 STEPS / REV
static constexpr int MOTOR_STEPS = 200;
constexpr uint8_t MICROSTEPS = 32; // 32
microstep microstep_param = microstep::THIRTY_SECOND;
constexpr uint32_t MIN_STEP_FREQ = (MIN_RPM * MOTOR_STEPS * MICROSTEPS) / 60;
constexpr uint32_t MAX_STEP_FREQ = (MAX_RPM * MOTOR_STEPS * MICROSTEPS) / 60;
int init_STEP_FREQUENCY = (MOTOR_RPM * MICROSTEPS * MOTOR_STEPS) / 60;

/*
  Stepper Motor Driver adapted from MotorDriver
*/
class StepperMotorDriver {
public:
  enum class direction { FORWARD, STOPPED, BACKWARD };

  StepperMotorDriver(const pin_pair_t &pins) : _pins(pins) {
    pinMode(_pins.first, OUTPUT);  // STEP
    pinMode(_pins.second, OUTPUT); // DIR

    digitalWrite(_pins.first, LOW);
    digitalWrite(_pins.second, LOW);
  }

  virtual ~StepperMotorDriver() {
    analogWrite(_pins.first, 0);

    digitalWrite(_pins.first, LOW);
    digitalWrite(_pins.second, LOW);

    pinMode(_pins.first, INPUT);
    pinMode(_pins.second, INPUT);
  }

  void set_speed(int16_t speed) {
    direction current_dir = direction::STOPPED;

    if (speed > 0)
      current_dir = direction::FORWARD;
    else if (speed < 0)
      current_dir = direction::BACKWARD;

    if (current_dir == direction::STOPPED) {
      analogWrite(_pins.first, 0);
      _prev_direction = current_dir;

      printf("STEPPER STOPPED\n");
      return;
    }

    // Set direction.
    if (current_dir != _prev_direction) {
      digitalWrite(_pins.second,
                   current_dir == direction::FORWARD ? HIGH : LOW);

      _prev_direction = current_dir;
    }

    // Convert int16 command magnitude to STEP frequency.
    const uint16_t magnitude =
        speed == std::numeric_limits<int16_t>::min() ? 32768 : std::abs(speed);

    const uint32_t step_frequency =
        map(magnitude, 0, 32768, MIN_STEP_FREQ, MAX_STEP_FREQ);

    // 50% duty-cycle square wave = continuous STEP pulses.
    analogWriteResolution(_pins.first, PWM_BITS);
    analogWriteFrequency(_pins.first, step_frequency);
    analogWrite(_pins.first, PWM_RES_HALF);

    printf("STEPPER SPEED: %ld Hz, %d RPM, DIR: %s\n", step_frequency,
           ((60 * step_frequency) / (MOTOR_STEPS * MICROSTEPS)),
           current_dir == direction::FORWARD ? "FORWARD" : "BACKWARD");
  }

private:
  direction _prev_direction = direction::STOPPED;

  pin_pair_t _pins;
};

StepperMotorDriver shaft_driver{SHAFT_PINS};

/*
  Test config VESC UART (CENTRIFUGE)
*/
float min_centrifuge_duty = 0;
float max_centrifgue_duty = 0.8;
float centrifuge_duty_set = 0;

/*
  VESC COMMS UART1
*/
static constexpr int VESC_UART_CH = 1;
static constexpr int TX1_PIN = GPIO_NUM_17;
static constexpr int RX1_PIN = GPIO_NUM_18;

/*
  VESC STATE FEEDBACK

  // 7 Values int16_t not read(14 byte)
  float avgMotorCurrent;
  float avgInputCurrent;
  float dutyCycleNow;
  long rpm;
  float inpVoltage;
  float ampHours;
  float ampHoursCharged;
  // 2 values int32_t not read (8 byte)
  long tachometer;
  long tachometerAbs;
*/

struct vesc_ramp_controller_t {
  float current_duty = 0.0f;
  float target_duty = 0.0f;

  float ramp_step = 0.01f;        // duty change per update
  uint32_t update_period_ms = 10; // 100 Hz ramp update
  uint32_t last_update_ms = 0;

  int uart_channel;
};

vesc_ramp_controller_t centrifuge = {.current_duty = 0.0f,
                                     .target_duty = 0.0f,
                                     .ramp_step = 0.01f,
                                     .update_period_ms = 50,
                                     .last_update_ms = 0,
                                     .uart_channel = VESC_UART_CH};

// VESC duty is ramped according to the currently set duty cycle.
void update_vesc_ramp(vesc_ramp_controller_t &controller) {
  const uint32_t now = millis();

  // not time for another control update yet.
  if (now - controller.last_update_ms < controller.update_period_ms) {
    return;
  }

  controller.last_update_ms = now;

  // ramp upward.
  if (controller.current_duty < controller.target_duty) {
    controller.current_duty = std::min(
        controller.current_duty + controller.ramp_step, controller.target_duty);
  }

  // ramp downward.
  else if (controller.current_duty > controller.target_duty) {
    controller.current_duty = std::max(
        controller.current_duty - controller.ramp_step, controller.target_duty);
  }

  VescUartSetDuty(controller.current_duty, controller.uart_channel);
}

struct bldcMeasure VESC_measured_values;
/*
  Test config INA238 Power Measurement

  20A ilim
  2 mOhm shunt

*/

INA238 INA(0x40);

constexpr uint8_t MAX_CURRENT = 20;
constexpr float SHUNT_RESISTANCE = 0.002;

// set global variables for measurement (for now)
float BUS_VOLTAGE = 0.0;
float BUS_CURRENT = 0.0;
float BUS_POWER = 0.0;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace hi_can;
using namespace hi_can::addressing;

std::optional<PacketManager> packet_manager;

constexpr standard_address_t DEVICE_ADDRESS{
    space_resources::SYSTEM_ID,
    space_resources::prospecting::SUBSYSTEM_ID,
    space_resources::prospecting::controller::DEVICE_ID,
};

void handle_drill_input(const Packet &packet);
void process_drill_input(
    const space_resources::prospecting::controller::drill_parameter &parameter,
    const int16_t &data);

void handle_shaft_input(const Packet &packet);
void process_shaft_input(
    const space_resources::prospecting::controller::shaft_parameter &parameter,
    const int16_t &data);

void handle_centrifuge_input(const Packet &packet);
void process_centrifuge_input(
    const space_resources::prospecting::controller::centrifuge_parameter
        &parameter,
    const int16_t &data);

void setup() {

  // setup UART port for VESC
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);

  // setup stepper motor driver
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);

  // setup OUTPUT MODE
  set_microstep(microstep_param);

  /*
  // setup INA238
  if (!INA.begin() )
  {
    printf(std::format("Could not setup INA238\n").c_str());
  }

  INA.setMaxCurrentShunt(MAX_CURRENT, SHUNT_RESISTANCE);
  */

  // setup interface
  auto &interface = TwaiInterface::get_instance(
      std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN), 0,
      filter_t{
          .address = static_cast<flagged_address_t>(DEVICE_ADDRESS),
          .mask = DEVICE_MASK,
      });
  packet_manager.emplace(interface);

  using namespace space_resources::prospecting::controller;

  // setup DRILL
  packet_manager->set_callback(
      filter_t{static_cast<flagged_address_t>(
          standard_address_t{DEVICE_ADDRESS, static_cast<uint8_t>(group::DRILL),
                             static_cast<uint8_t>(drill_parameter::SPEED)})},
      {
          .data_callback = handle_drill_input,
      });

  // setup SHAFT
  packet_manager->set_callback(
      filter_t{static_cast<flagged_address_t>(
          standard_address_t{DEVICE_ADDRESS, static_cast<uint8_t>(group::SHAFT),
                             static_cast<uint8_t>(shaft_parameter::SPEED)})},
      {
          .data_callback = handle_shaft_input,
      });

  packet_manager->set_callback(
      filter_t{static_cast<flagged_address_t>(
          standard_address_t{DEVICE_ADDRESS, static_cast<uint8_t>(group::SHAFT),
                             static_cast<uint8_t>(shaft_parameter::ROTATION)})},
      {
          .data_callback = handle_shaft_input,
      });

  // setup CENTRIFUGE
  packet_manager->set_callback(
      filter_t{static_cast<flagged_address_t>(standard_address_t{
          DEVICE_ADDRESS, static_cast<uint8_t>(group::CENTRIFUGE),
          static_cast<uint8_t>(centrifuge_parameter::ROTATE_SPEED)})},
      {
          .data_callback = handle_centrifuge_input,
      });

  // setup CENTRIFUGE TX
  // TODO: Update to transmission generator to general Space Resources state
  // vector.

  using namespace parameters::drive::vesc;
  packet_manager->set_transmission_config(
      static_cast<flagged_address_t>(standard_address_t{
          DEVICE_ADDRESS, static_cast<uint8_t>(group::CENTRIFUGE),
          static_cast<uint8_t>(centrifuge_parameter::RPM)}),
      {
          .generator =
              [&]() {
                if (!VescUartGetValue(VESC_measured_values, VESC_UART_CH)) {
                  printf(std::format("ERROR: Failed to read VESC: {}\n", 0)
                             .c_str());
                  return status_1_t{}.serialize_data();
                }
                printf(std::format("PASS: Read VESC: {}\n", 1).c_str());

                status_1_t status{};
                status.rpm = VESC_measured_values.rpm;
                status.current = VESC_measured_values.avgInputCurrent;
                status.duty_cycle = VESC_measured_values.dutyCycleNow;

                return status.serialize_data();
              },
          .interval = 1000ms,
          .should_transmit_immediately = true,
      });
}

void loop() {
  packet_manager->handle();

  // VESC duty is ramped according to the currently set duty cycle.
  update_vesc_ramp(centrifuge);

  // update global power variables for transmission
  //BUS_VOLTAGE = INA.getBusVoltage();
  //BUS_CURRENT = INA.getAmpere();
  //BUS_POWER = BUS_CURRENT * BUS_VOLTAGE;

  delay(1);
}

void handle_drill_input(const Packet &packet) {
  using namespace space_resources::prospecting::controller;

  printf("Handle DRILL Input\n");

  try {
    // Retrieve full address from CAN
    standard_address_t address{packet.get_address().address};

    // Retrieve data from CAN
    const auto &data_bytes = packet.get_data();

    if (data_bytes.size() != sizeof(int16_t)) {
      printf("Invalid payload size: expected %u bytes, received %u bytes\n",
             static_cast<unsigned>(sizeof(int16_t)),
             static_cast<unsigned>(data_bytes.size()));
      return;
    }

    int16_t value;
    memcpy(&value, data_bytes.data(), sizeof(value));

    printf(std::format("RX group={:#04x}, parameter={:#04x}, data={}\n",
                       static_cast<uint8_t>(address.group),
                       static_cast<uint8_t>(address.parameter), value)
               .c_str());

    process_drill_input(static_cast<drill_parameter>(address.parameter), value);

  } catch (const std::exception &e) {
    printf(std::format("Failed to parse drill input: {}\n", e.what()).c_str());
  }
}

void process_drill_input(
    const space_resources::prospecting::controller::drill_parameter &parameter,
    const int16_t &data) {

  using namespace space_resources::prospecting::controller;

  switch (parameter) {

  case drill_parameter::SPEED:
    printf(std::format("DRILL SPEED: {}\n", data).c_str());
    drill_driver.set_speed(data);
    break;

  default:
    printf(std::format("Unknown drill parameter: {:#04x}\n",
                       static_cast<uint8_t>(parameter))
               .c_str());
    break;
  }
}

void handle_shaft_input(const Packet &packet) {
  using namespace space_resources::prospecting::controller;
  printf(std::format("Handle SHAFT Input \n").c_str());

  try {
    // retrieve full address from CAN
    standard_address_t address{packet.get_address().address};

    // retrieve data from CAN
    const auto &data_bytes = packet.get_data();
    if (data_bytes.size() != sizeof(int16_t)) {
      printf("Invalid payload size: expected %u bytes, received %u bytes\n",
             static_cast<unsigned>(sizeof(int16_t)),
             static_cast<unsigned>(data_bytes.size()));
      return;
    }
    int16_t value;
    memcpy(&value, data_bytes.data(), sizeof(value));

    printf(std::format("RX group={:#04x}, parameter={:#04x}, data={}\n",
                       static_cast<uint8_t>(address.group),
                       static_cast<uint8_t>(address.parameter), value)
               .c_str());
    process_shaft_input(static_cast<shaft_parameter>(address.parameter), value);

  } catch (const std::exception &e) {
    printf(std::format("Failed to parse test input: {}\n", e.what()).c_str());
  }
}

void process_shaft_input(
    const space_resources::prospecting::controller::shaft_parameter &parameter,
    const int16_t &data) {
  using namespace space_resources::prospecting::controller;
  switch (parameter) {

  case shaft_parameter::SPEED:
    shaft_driver.set_speed(data);
    break;

  default:
    break;
  }
}

void handle_centrifuge_input(const Packet &packet) {
  using namespace space_resources::prospecting::controller;

  printf("Handle CENTRIFUGE Input\n");

  try {
    // Retrieve full address from CAN
    standard_address_t address{packet.get_address().address};

    // Retrieve data from CAN
    const auto &data_bytes = packet.get_data();

    if (data_bytes.size() != sizeof(int16_t)) {
      printf("Invalid payload size: expected %u bytes, received %u bytes\n",
             static_cast<unsigned>(sizeof(int16_t)),
             static_cast<unsigned>(data_bytes.size()));
      return;
    }

    int16_t value;
    memcpy(&value, data_bytes.data(), sizeof(value));

    printf(std::format("RX group={:#04x}, parameter={:#04x}, data={}\n",
                       static_cast<uint8_t>(address.group),
                       static_cast<uint8_t>(address.parameter), value)
               .c_str());

    process_centrifuge_input(
        static_cast<centrifuge_parameter>(address.parameter), value);

  } catch (const std::exception &e) {
    printf(std::format("Failed to parse centrifuge input: {}\n", e.what())
               .c_str());
  }
}

void process_centrifuge_input(
    const space_resources::prospecting::controller::centrifuge_parameter
        &parameter,
    const int16_t &data) {

  using namespace space_resources::prospecting::controller;

  switch (parameter) {

  case centrifuge_parameter::ROTATE_SPEED:

    centrifuge_duty_set =
        static_cast<float>(data) /
        static_cast<float>(std::numeric_limits<int16_t>::max());

    centrifuge.target_duty = centrifuge_duty_set;

    printf(std::format("CENTRIFUGE DUTY: {}\n", centrifuge_duty_set).c_str());

    break;

  default:
    printf(std::format("Unknown centrifuge parameter: {:#04x}\n",
                       static_cast<uint8_t>(parameter))
               .c_str());
    break;
  }
}
