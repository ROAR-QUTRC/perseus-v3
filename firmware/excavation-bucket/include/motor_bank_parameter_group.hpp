#include "hi_can_parameter.hpp"
#include "motor_bank.hpp"

class MotorBankParameterGroup : public hi_can::parameters::ParameterGroup
{
public:
    MotorBankParameterGroup(hi_can::addressing::excavation::bucket::controller::group bank_group, MotorBank& motor_bank);

private:
    const hi_can::addressing::excavation::bucket::controller::group _bank_group;
    MotorBank& _motor_bank;
};
