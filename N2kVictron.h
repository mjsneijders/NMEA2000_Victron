#ifndef N2K_VICTRON_H
#define N2K_VICTRON_H

#include <N2kMsg.h>
#include <N2kTypes.h>
#include <stddef.h>
#include <stdint.h>

// Codecs only: setters construct messages; they never transmit them.
// All numeric measurements use V, A, Hz, W, VA, var and kWh, or N2kDoubleNA.
// False means wrong PGN, malformed input or an out-of-range value.
// On failure, the output message/data is unchanged.

enum class N2kVictronRegister : uint16_t {
  Request = 0x0001,
  Acknowledgement = 0x0002,
  DeviceMode = 0x0200,
  DeviceState = 0x0201,
  RemoteControl = 0x0202,
  ActiveCurrentLimit = 0x0203,
  ActiveInput = 0x0204,
  AcIn1Limit = 0x0210,
  AcIn1LimitMin = 0x0211,
  AcIn1LimitMax = 0x0212,
  AcIn1LimitInternal = 0x0213,
  AcIn1LimitRemote = 0x0214,
  AcIn2Limit = 0x0220,
  AcIn2LimitMin = 0x0221,
  AcIn2LimitMax = 0x0222,
  AcIn2LimitInternal = 0x0223,
  AcIn2LimitRemote = 0x0224,
  ConsumedAh = 0xeeff
};

enum class N2kVictronMode : uint8_t {
  ChargerOnly = 1, InverterOnly = 2, On = 3, Off = 4, Eco = 5
};

constexpr uint32_t N2kVictronRemoteAcIn1 = 1UL << 0;
constexpr uint32_t N2kVictronRemoteMode = 1UL << 1;
constexpr uint32_t N2kVictronRemoteAcIn2 = 1UL << 2;
constexpr unsigned long N2kVictronSingleFramePGN = 61184;
constexpr unsigned long N2kVictronFastPacketPGN = 126720;

struct N2kVictronVreg {
  uint16_t registerId{0};
  size_t dataLength{0};
  uint8_t data[219]{};
};

// PGN 61184 (up to 4 data bytes, zero-padded to 8) or 126720 (up to 219).
// Broadcast destination 255 is allowed for register announcements and reads.
bool SetN2kVictronVreg(tN2kMsg &msg, uint8_t destination, uint16_t registerId,
                      const uint8_t *data, size_t length, bool fastPacket = false);
bool ParseN2kVictronVreg(const tN2kMsg &msg, N2kVictronVreg &out);
bool SetN2kVictronRegisterRequest(tN2kMsg &msg, uint8_t destination,
                                 uint16_t registerId, uint16_t mask = 0xffff);
bool ParseN2kVictronRegisterRequest(const tN2kMsg &msg, uint16_t &registerId, uint16_t &mask);
bool SetN2kVictronRegisterAck(tN2kMsg &msg, uint8_t destination,
                             uint16_t registerId, uint16_t ackCode);
bool ParseN2kVictronRegisterAck(const tN2kMsg &msg, uint16_t &registerId, uint16_t &ackCode);

// Register values/announcements, including device state, remote-control bitmask,
// current-limit registers and raw two's-complement consumed Ah (0.1 Ah).
bool SetN2kVictronRegisterValue(tN2kMsg &msg, uint8_t destination,
                               uint16_t registerId, uint32_t value);
bool ParseN2kVictronRegisterValue(const tN2kMsg &msg, uint16_t &registerId, uint32_t &value);

// Addressed control commands only (destination 0..253). Current limit is in A;
// device-specific min/max and control ownership must be checked by the caller.
bool SetN2kVictronDeviceMode(tN2kMsg &msg, uint8_t destination, N2kVictronMode mode);
bool SetN2kVictronRemoteControl(tN2kMsg &msg, uint8_t destination, uint32_t features);
bool SetN2kVictronCurrentLimit(tN2kMsg &msg, uint8_t destination, double amps, uint8_t input = 1);

// J1939 Utility AC PGNs. Source/ISO NAME distinguishes AC input from output.
// Phase A = L1, B = L2, C = L3.
enum class N2kUtilityAcPGN : unsigned long {
  Energy = 65005,
  ReactiveC = 65006, PowerC = 65007, BasicC = 65008,
  ReactiveB = 65009, PowerB = 65010, BasicB = 65011,
  ReactiveA = 65012, PowerA = 65013, BasicA = 65014,
  ReactiveTotal = 65015, PowerTotal = 65016, BasicAverage = 65017
};

struct N2kAcBasic {
  double lineLineVoltage{N2kDoubleNA};
  double lineNeutralVoltage{N2kDoubleNA};
  double frequency{N2kDoubleNA};
  double current{N2kDoubleNA};
};
struct N2kAcPower {
  double realPower{N2kDoubleNA};
  double apparentPower{N2kDoubleNA};
};
enum class N2kPowerFactorLag : uint8_t { Leading = 0, Lagging = 1, Error = 2, Unavailable = 3 };
struct N2kAcReactivePower {
  double reactivePower{N2kDoubleNA};
  double powerFactor{N2kDoubleNA};
  N2kPowerFactorLag lagging{N2kPowerFactorLag::Unavailable};
};
struct N2kAcEnergy {
  double exportKwh{N2kDoubleNA};
  double importKwh{N2kDoubleNA};
};

bool SetN2kUtilityAcBasic(tN2kMsg &msg, N2kUtilityAcPGN pgn, const N2kAcBasic &data);
bool ParseN2kUtilityAcBasic(const tN2kMsg &msg, N2kAcBasic &out);
bool SetN2kUtilityAcPower(tN2kMsg &msg, N2kUtilityAcPGN pgn, const N2kAcPower &data);
bool ParseN2kUtilityAcPower(const tN2kMsg &msg, N2kAcPower &out);
bool SetN2kUtilityAcReactivePower(tN2kMsg &msg, N2kUtilityAcPGN pgn, const N2kAcReactivePower &data);
bool ParseN2kUtilityAcReactivePower(const tN2kMsg &msg, N2kAcReactivePower &out);
bool SetN2kUtilityAcEnergy(tN2kMsg &msg, const N2kAcEnergy &data);
bool ParseN2kUtilityAcEnergy(const tN2kMsg &msg, N2kAcEnergy &out);

// Legacy PGN 127509; state is not the Victron VREG device-state enumeration.
enum class N2kInverterState : uint8_t {
  Invert = 0, AcPassThrough = 1, LoadSense = 2, Fault = 3, Disabled = 4,
  Unavailable = 15
};
struct N2kInverterStatus {
  uint8_t instance{0xff};
  uint8_t acInstance{0xff};
  uint8_t dcInstance{0xff};
  N2kInverterState state{N2kInverterState::Unavailable};
  tN2kOnOff enabled{N2kOnOff_Unavailable};
};
bool SetN2kPGN127509(tN2kMsg &msg, const N2kInverterStatus &data);
bool ParseN2kPGN127509(const tN2kMsg &msg, N2kInverterStatus &out);

// Legacy AC Input/Output Status 127503/127504 (fast packet, 20 bytes).
// line: 0..2 or 3=unavailable. quality: input acceptability (2 bits) or
// output waveform (3 bits), using the standard raw lookup values.
// quality 0xff encodes the appropriate unavailable value for either PGN.
struct N2kLegacyAcStatus {
  uint8_t instance{0xff};
  uint8_t numberOfLines{0xff};
  uint8_t line{3};
  uint8_t quality{0xff};
  double voltage{N2kDoubleNA};
  double current{N2kDoubleNA};
  double frequency{N2kDoubleNA};
  double breakerSize{N2kDoubleNA};
  double realPower{N2kDoubleNA};
  double reactivePower{N2kDoubleNA};
  double powerFactor{N2kDoubleNA};
};
bool SetN2kLegacyAcStatus(tN2kMsg &msg, unsigned long pgn, const N2kLegacyAcStatus &data);
bool ParseN2kLegacyAcStatus(const tN2kMsg &msg, N2kLegacyAcStatus &out);

// Modern phase-specific AC PGNs 127744..127749. Values are not J1939-biased.
enum class N2kAcPhase : uint8_t { A = 0, B = 1, C = 2 };
struct N2kAcPowerCurrent {
  uint8_t sid{0xff};
  uint8_t connection{0xff};
  double current{N2kDoubleNA};
  double power{N2kDoubleNA};
};
struct N2kAcVoltageFrequency {
  uint8_t sid{0xff};
  uint8_t connection{0xff};
  double lineNeutralVoltage{N2kDoubleNA};
  double lineLineVoltage{N2kDoubleNA};
  double frequency{N2kDoubleNA};
};
bool SetN2kAcPowerCurrent(tN2kMsg &msg, N2kAcPhase phase, const N2kAcPowerCurrent &data);
bool ParseN2kAcPowerCurrent(const tN2kMsg &msg, N2kAcPowerCurrent &out);
bool SetN2kAcVoltageFrequency(tN2kMsg &msg, N2kAcPhase phase, const N2kAcVoltageFrequency &data);
bool ParseN2kAcVoltageFrequency(const tN2kMsg &msg, N2kAcVoltageFrequency &out);

#endif
