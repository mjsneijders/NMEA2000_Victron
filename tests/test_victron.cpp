#include "N2kVictron.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <limits>

static void bytes(const tN2kMsg &msg, unsigned long pgn, const uint8_t *expected, size_t length) {
  assert(msg.PGN == pgn);
  assert(msg.DataLen == static_cast<int>(length));
  assert(std::memcmp(msg.Data, expected, length) == 0);
}

static void near(double actual, double expected, double tolerance = 0.000001) {
  assert(std::fabs(actual - expected) <= tolerance);
}

static void testVreg() {
  tN2kMsg msg;
  assert(SetN2kVictronRegisterRequest(msg, 32, 0x0200));
  const uint8_t request[] = {0x66, 0x99, 1, 0, 0, 2, 0xff, 0xff};
  bytes(msg, 61184, request, sizeof(request));
  assert(msg.Priority == 7 && msg.Destination == 32);
  uint16_t reg = 0, mask = 0;
  assert(ParseN2kVictronRegisterRequest(msg, reg, mask));
  assert(reg == 0x0200 && mask == 0xffff);
  uint32_t value = 0;
  assert(!ParseN2kVictronRegisterValue(msg, reg, value));
  assert(SetN2kVictronRegisterRequest(msg, 255, 0x0200, 0xff00));
  assert(ParseN2kVictronRegisterRequest(msg, reg, mask) && mask == 0xff00);

  assert(SetN2kVictronDeviceMode(msg, 32, N2kVictronMode::Off));
  const uint8_t off[] = {0x66, 0x99, 0, 2, 4, 0, 0, 0};
  bytes(msg, 61184, off, sizeof(off));
  assert(ParseN2kVictronRegisterValue(msg, reg, value) && reg == 0x0200 && value == 4);
  assert(!SetN2kVictronDeviceMode(msg, 255, N2kVictronMode::Off));
  assert(!SetN2kVictronDeviceMode(msg, 32, static_cast<N2kVictronMode>(0)));
  bytes(msg, 61184, off, sizeof(off));
  assert(SetN2kVictronRemoteControl(msg, 32, N2kVictronRemoteAcIn1 | N2kVictronRemoteMode));
  assert(ParseN2kVictronRegisterValue(msg, reg, value) && reg == 0x0202 && value == 3);
  assert(!SetN2kVictronRemoteControl(msg, 255, 3));
  assert(SetN2kVictronCurrentLimit(msg, 32, 16.0));
  const uint8_t limit[] = {0x66, 0x99, 0x14, 2, 0xa0, 0, 0, 0};
  bytes(msg, 61184, limit, sizeof(limit));
  assert(SetN2kVictronCurrentLimit(msg, 32, 12.3, 2));
  assert(ParseN2kVictronRegisterValue(msg, reg, value) && reg == 0x0224 && value == 123);
  assert(!SetN2kVictronCurrentLimit(msg, 32, -1));
  assert(!SetN2kVictronCurrentLimit(msg, 32, N2kDoubleNA));
  assert(!SetN2kVictronCurrentLimit(msg, 32, 6553.5));
  assert(!SetN2kVictronCurrentLimit(msg, 32, 16, 3));
  assert(!SetN2kVictronCurrentLimit(msg, 255, 16));
  assert(SetN2kVictronRegisterAck(msg, 17, 0x0214, 0xc001));
  const uint8_t ack[] = {0x66, 0x99, 2, 0, 0x14, 2, 1, 0xc0};
  bytes(msg, 61184, ack, sizeof(ack));
  assert(ParseN2kVictronRegisterAck(msg, reg, mask) && reg == 0x0214 && mask == 0xc001);
  assert(!ParseN2kVictronRegisterRequest(msg, reg, mask));

  uint8_t payload[220];
  for (size_t i = 0; i < sizeof(payload); ++i) payload[i] = static_cast<uint8_t>(i);
  assert(SetN2kVictronVreg(msg, 255, 0xeeff, payload, 219, true));
  assert(msg.PGN == 126720 && msg.DataLen == 223);
  N2kVictronVreg parsed;
  assert(ParseN2kVictronVreg(msg, parsed));
  assert(parsed.registerId == 0xeeff && parsed.dataLength == 219);
  assert(std::memcmp(payload, parsed.data, 219) == 0);
  assert(!SetN2kVictronVreg(msg, 32, 0xeeff, payload, 220, true));
  assert(!SetN2kVictronVreg(msg, 32, 0xeeff, payload, 5));
  assert(!SetN2kVictronVreg(msg, 254, 0xeeff, payload, 4));
  assert(!SetN2kVictronVreg(msg, 32, 0xeeff, nullptr, 1));
  assert(!SetN2kVictronVreg(msg, 32, 0, payload, 4));
  msg.Data[0] = 0;
  assert(!ParseN2kVictronVreg(msg, parsed));
  assert(parsed.dataLength == 219);
  assert(SetN2kVictronRegisterValue(msg, 255, 0x0201, 9));
  assert(ParseN2kVictronRegisterValue(msg, reg, value) && reg == 0x0201 && value == 9);
  for (int length = 0; length < 4; ++length) {
    msg.DataLen = length;
    assert(!ParseN2kVictronVreg(msg, parsed));
  }
  msg.DataLen = 9;
  assert(!ParseN2kVictronVreg(msg, parsed));
  assert(SetN2kVictronVreg(msg, 255, 0xeeff, payload, 1));
  assert(msg.Data[4] == 0 && msg.Data[5] == 0 && msg.Data[6] == 0 && msg.Data[7] == 0);
}

static void testUtility() {
  tN2kMsg msg;
  N2kAcBasic basic;
  basic.lineLineVoltage = N2kDoubleNA;
  basic.lineNeutralVoltage = 230;
  basic.frequency = 50;
  basic.current = 12;
  const uint8_t expected[] = {0xff, 0xff, 0xe6, 0, 0, 0x19, 12, 0};
  for (auto pgn : {N2kUtilityAcPGN::BasicA, N2kUtilityAcPGN::BasicB,
                   N2kUtilityAcPGN::BasicC, N2kUtilityAcPGN::BasicAverage}) {
    assert(SetN2kUtilityAcBasic(msg, pgn, basic));
    bytes(msg, static_cast<unsigned long>(pgn), expected, sizeof(expected));
    N2kAcBasic parsed;
    assert(ParseN2kUtilityAcBasic(msg, parsed));
    assert(N2kIsNA(parsed.lineLineVoltage));
    near(parsed.lineNeutralVoltage, 230);
    near(parsed.frequency, 50);
    near(parsed.current, 12);
    --msg.DataLen;
    assert(!ParseN2kUtilityAcBasic(msg, parsed));
  }
  assert(!SetN2kUtilityAcBasic(msg, N2kUtilityAcPGN::PowerA, basic));
  basic.frequency = std::numeric_limits<double>::infinity();
  assert(!SetN2kUtilityAcBasic(msg, N2kUtilityAcPGN::BasicA, basic));

  N2kAcPower power;
  power.realPower = -500;
  power.apparentPower = 1000;
  const uint8_t powerBytes[] = {0x0c, 0x92, 0x35, 0x77, 0xe8, 0x97, 0x35, 0x77};
  for (auto pgn : {N2kUtilityAcPGN::PowerA, N2kUtilityAcPGN::PowerB,
                   N2kUtilityAcPGN::PowerC, N2kUtilityAcPGN::PowerTotal}) {
    assert(SetN2kUtilityAcPower(msg, pgn, power));
    bytes(msg, static_cast<unsigned long>(pgn), powerBytes, sizeof(powerBytes));
    N2kAcPower parsed;
    assert(ParseN2kUtilityAcPower(msg, parsed));
    near(parsed.realPower, -500);
    near(parsed.apparentPower, 1000);
    --msg.DataLen;
    assert(!ParseN2kUtilityAcPower(msg, parsed));
  }
  power.realPower = -2000000000.0;
  power.apparentPower = 2294967292.0;
  assert(SetN2kUtilityAcPower(msg, N2kUtilityAcPGN::PowerA, power));
  assert(msg.Data[0] == 0 && msg.Data[4] == 0xfc && msg.Data[7] == 0xff);
  N2kAcPower parsedPower;
  assert(ParseN2kUtilityAcPower(msg, parsedPower));
  near(parsedPower.realPower, power.realPower);
  near(parsedPower.apparentPower, power.apparentPower);
  power.realPower = N2kDoubleNA;
  assert(SetN2kUtilityAcPower(msg, N2kUtilityAcPGN::PowerA, power));
  assert(ParseN2kUtilityAcPower(msg, parsedPower) && N2kIsNA(parsedPower.realPower));
  power.realPower = std::numeric_limits<double>::quiet_NaN();
  assert(!SetN2kUtilityAcPower(msg, N2kUtilityAcPGN::PowerA, power));

  N2kAcReactivePower reactive;
  reactive.reactivePower = 0;
  reactive.powerFactor = 1;
  reactive.lagging = N2kPowerFactorLag::Lagging;
  const uint8_t reactiveBytes[] = {0, 0x94, 0x35, 0x77, 0, 0x80, 0xfd, 0xff};
  for (auto pgn : {N2kUtilityAcPGN::ReactiveA, N2kUtilityAcPGN::ReactiveB,
                   N2kUtilityAcPGN::ReactiveC, N2kUtilityAcPGN::ReactiveTotal}) {
    assert(SetN2kUtilityAcReactivePower(msg, pgn, reactive));
    bytes(msg, static_cast<unsigned long>(pgn), reactiveBytes, sizeof(reactiveBytes));
    N2kAcReactivePower parsed;
    assert(ParseN2kUtilityAcReactivePower(msg, parsed));
    near(parsed.reactivePower, 0);
    near(parsed.powerFactor, 1);
    assert(parsed.lagging == N2kPowerFactorLag::Lagging);
    --msg.DataLen;
    assert(!ParseN2kUtilityAcReactivePower(msg, parsed));
  }
  N2kAcEnergy energy;
  energy.exportKwh = 25;
  energy.importKwh = 100;
  assert(SetN2kUtilityAcEnergy(msg, energy));
  const uint8_t energyBytes[] = {25, 0, 0, 0, 100, 0, 0, 0};
  bytes(msg, 65005, energyBytes, sizeof(energyBytes));
  N2kAcEnergy parsedEnergy;
  assert(ParseN2kUtilityAcEnergy(msg, parsedEnergy));
  near(parsedEnergy.exportKwh, 25);
  near(parsedEnergy.importKwh, 100);
  --msg.DataLen;
  assert(!ParseN2kUtilityAcEnergy(msg, parsedEnergy));
}

static void testNmeaAc() {
  tN2kMsg msg;
  N2kInverterStatus inverter;
  inverter.instance = 1;
  inverter.acInstance = 2;
  inverter.dcInstance = 239;
  inverter.state = N2kInverterState::AcPassThrough;
  inverter.enabled = N2kOnOff_On;
  assert(SetN2kPGN127509(msg, inverter));
  const uint8_t inverterBytes[] = {1, 2, 239, 0xd1};
  bytes(msg, 127509, inverterBytes, sizeof(inverterBytes));
  N2kInverterStatus parsedInverter;
  assert(ParseN2kPGN127509(msg, parsedInverter));
  assert(parsedInverter.state == inverter.state && parsedInverter.dcInstance == 239);
  msg.DataLen = 3;
  assert(!ParseN2kPGN127509(msg, parsedInverter));

  N2kAcPowerCurrent power;
  power.sid = 1;
  power.connection = 2;
  power.current = 12.3;
  power.power = -500;
  const uint8_t expected[] = {1, 2, 123, 0, 0x0c, 0xfe, 0xff, 0xff};
  N2kAcVoltageFrequency voltage;
  voltage.sid = 1;
  voltage.connection = 2;
  voltage.lineNeutralVoltage = 230;
  voltage.lineLineVoltage = 400;
  voltage.frequency = 50;
  const uint8_t voltageBytes[] = {1, 2, 0xfc, 8, 0xa0, 15, 0xf4, 1};
  for (auto phase : {N2kAcPhase::A, N2kAcPhase::B, N2kAcPhase::C}) {
    assert(SetN2kAcPowerCurrent(msg, phase, power));
    bytes(msg, 127744 + static_cast<uint8_t>(phase), expected, sizeof(expected));
    N2kAcPowerCurrent parsed;
    assert(ParseN2kAcPowerCurrent(msg, parsed));
    near(parsed.current, 12.3);
    near(parsed.power, -500);
    --msg.DataLen;
    assert(!ParseN2kAcPowerCurrent(msg, parsed));
    assert(SetN2kAcVoltageFrequency(msg, phase, voltage));
    bytes(msg, 127747 + static_cast<uint8_t>(phase), voltageBytes, sizeof(voltageBytes));
    N2kAcVoltageFrequency parsedVoltage;
    assert(ParseN2kAcVoltageFrequency(msg, parsedVoltage));
    near(parsedVoltage.lineLineVoltage, 400);
    near(parsedVoltage.lineNeutralVoltage, 230);
    near(parsedVoltage.frequency, 50);
    --msg.DataLen;
    assert(!ParseN2kAcVoltageFrequency(msg, parsedVoltage));
  }
  for (double value : {-2147483648.0, 2147483644.0, N2kDoubleNA}) {
    power.power = value;
    assert(SetN2kAcPowerCurrent(msg, N2kAcPhase::A, power));
    N2kAcPowerCurrent parsed;
    assert(ParseN2kAcPowerCurrent(msg, parsed));
    near(parsed.power, value);
  }
  assert(!SetN2kAcPowerCurrent(msg, static_cast<N2kAcPhase>(3), power));
  assert(!SetN2kAcVoltageFrequency(msg, static_cast<N2kAcPhase>(3), voltage));
  N2kLegacyAcStatus legacy;
  legacy.instance = 1;
  legacy.numberOfLines = 1;
  legacy.line = 0;
  legacy.quality = 1;
  legacy.voltage = 230;
  legacy.current = 12.3;
  legacy.frequency = 50;
  legacy.breakerSize = 16;
  legacy.realPower = 2800;
  legacy.reactivePower = 10;
  legacy.powerFactor = -0.95;
  for (unsigned long pgn : {127503UL, 127504UL}) {
    assert(SetN2kLegacyAcStatus(msg, pgn, legacy));
    const uint8_t expectedLegacy[] = {1, 1, static_cast<uint8_t>(pgn == 127503 ? 0xf4 : 0xe4),
      0xd8, 0x59, 123, 0, 0x88, 0x13, 0xa0, 0, 0xf0, 10, 0, 0, 10, 0, 0, 0, 0xa1};
    bytes(msg, pgn, expectedLegacy, sizeof(expectedLegacy));
    N2kLegacyAcStatus parsed;
    assert(ParseN2kLegacyAcStatus(msg, parsed));
    near(parsed.voltage, 230);
    near(parsed.powerFactor, -0.95);
    --msg.DataLen;
    assert(!ParseN2kLegacyAcStatus(msg, parsed));
  }
}

static void testUnavailableAndInvalid() {
  tN2kMsg msg;
  N2kAcBasic basic;
  assert(SetN2kUtilityAcBasic(msg, N2kUtilityAcPGN::BasicA, basic));
  N2kAcBasic parsedBasic;
  assert(ParseN2kUtilityAcBasic(msg, parsedBasic));
  assert(N2kIsNA(parsedBasic.current) && N2kIsNA(parsedBasic.frequency));
  msg.Data[0] = 0xfe;
  assert(ParseN2kUtilityAcBasic(msg, parsedBasic) && N2kIsNA(parsedBasic.lineLineVoltage));

  N2kAcReactivePower reactive;
  assert(SetN2kUtilityAcReactivePower(msg, N2kUtilityAcPGN::ReactiveA, reactive));
  assert(ParseN2kUtilityAcReactivePower(msg, reactive));
  assert(N2kIsNA(reactive.reactivePower) && N2kIsNA(reactive.powerFactor));
  reactive.powerFactor = 1.1;
  assert(!SetN2kUtilityAcReactivePower(msg, N2kUtilityAcPGN::ReactiveA, reactive));
  N2kAcEnergy energy;
  assert(SetN2kUtilityAcEnergy(msg, energy));
  assert(ParseN2kUtilityAcEnergy(msg, energy));
  assert(N2kIsNA(energy.importKwh) && N2kIsNA(energy.exportKwh));
  energy.exportKwh = -1;
  assert(!SetN2kUtilityAcEnergy(msg, energy));

  N2kInverterStatus inverter;
  assert(SetN2kPGN127509(msg, inverter));
  assert(msg.Data[3] == 0xff);
  assert(ParseN2kPGN127509(msg, inverter));
  assert(inverter.state == N2kInverterState::Unavailable);
  N2kAcPowerCurrent power;
  assert(SetN2kAcPowerCurrent(msg, N2kAcPhase::A, power));
  assert(ParseN2kAcPowerCurrent(msg, power));
  assert(N2kIsNA(power.current) && N2kIsNA(power.power));
  msg.Data[4] = 0xfe;
  assert(ParseN2kAcPowerCurrent(msg, power) && N2kIsNA(power.power));
  N2kAcVoltageFrequency voltage;
  assert(SetN2kAcVoltageFrequency(msg, N2kAcPhase::A, voltage));
  assert(ParseN2kAcVoltageFrequency(msg, voltage));
  assert(N2kIsNA(voltage.frequency) && N2kIsNA(voltage.lineNeutralVoltage));
  N2kLegacyAcStatus legacy;
  for (unsigned long pgn : {127503UL, 127504UL}) {
    assert(SetN2kLegacyAcStatus(msg, pgn, legacy));
    assert(msg.Data[2] == 0xff);
    N2kLegacyAcStatus parsed;
    assert(ParseN2kLegacyAcStatus(msg, parsed));
    assert(N2kIsNA(parsed.current) && N2kIsNA(parsed.powerFactor));
  }

  msg.PGN = 1;
  N2kVictronVreg vreg;
  N2kAcPower acPower;
  assert(!ParseN2kVictronVreg(msg, vreg));
  assert(!ParseN2kUtilityAcBasic(msg, basic));
  assert(!ParseN2kUtilityAcPower(msg, acPower));
  assert(!ParseN2kUtilityAcReactivePower(msg, reactive));
  assert(!ParseN2kUtilityAcEnergy(msg, energy));
  assert(!ParseN2kPGN127509(msg, inverter));
  assert(!ParseN2kAcPowerCurrent(msg, power));
  assert(!ParseN2kAcVoltageFrequency(msg, voltage));
  assert(!ParseN2kLegacyAcStatus(msg, legacy));
}

int main() {
  testVreg();
  testUtility();
  testNmeaAc();
  testUnavailableAndInvalid();
  std::puts("Victron/AC codecs: all tests passed");
}
