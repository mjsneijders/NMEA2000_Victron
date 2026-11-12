#include "N2kVictron.h"

#include <cmath>
#include <cstring>

namespace {
bool number(double value, double minimum, double maximum) {
  return N2kIsNA(value) || (std::isfinite(value) && value >= minimum && value <= maximum);
}

void start(tN2kMsg &msg, unsigned long pgn, uint8_t priority, uint8_t destination = 255) {
  msg.SetPGN(pgn);
  msg.Priority = priority;
  msg.Destination = destination;
}

// J1939 uses offset-binary power, unlike the signed NMEA AC power fields.
void addUnsigned(tN2kMsg &msg, double value, double scale, double offset, bool wide = false) {
  const uint32_t raw = N2kIsNA(value) ? (wide ? 0xffffffffUL : 0xffffUL)
                                    : static_cast<uint32_t>(std::round((value - offset) / scale));
  if (wide) msg.Add4ByteUInt(raw);
  else msg.Add2ByteUInt(static_cast<uint16_t>(raw));
}

double getUnsigned(const tN2kMsg &msg, int &index, double scale, double offset, bool wide = false) {
  const uint32_t raw = wide ? msg.Get4ByteUInt(index) : msg.Get2ByteUInt(index);
  if (raw > (wide ? 0xfffffffcUL : 0xfffcUL)) return N2kDoubleNA;
  return raw * scale + offset;
}

bool basicPgn(unsigned long pgn) {
  return pgn == 65008 || pgn == 65011 || pgn == 65014 || pgn == 65017;
}
bool powerPgn(unsigned long pgn) {
  return pgn == 65007 || pgn == 65010 || pgn == 65013 || pgn == 65016;
}
bool reactivePgn(unsigned long pgn) {
  return pgn == 65006 || pgn == 65009 || pgn == 65012 || pgn == 65015;
}
bool vregPair(const tN2kMsg &msg, uint16_t expected, uint16_t &first, uint16_t &second) {
  N2kVictronVreg value;
  if (!ParseN2kVictronVreg(msg, value) || value.registerId != expected || value.dataLength != 4) return false;
  first = static_cast<uint16_t>(value.data[0] | (value.data[1] << 8));
  second = static_cast<uint16_t>(value.data[2] | (value.data[3] << 8));
  return true;
}
bool setVregPair(tN2kMsg &msg, uint8_t destination, uint16_t reg, uint16_t first, uint16_t second) {
  const uint8_t data[] = {static_cast<uint8_t>(first), static_cast<uint8_t>(first >> 8),
                          static_cast<uint8_t>(second), static_cast<uint8_t>(second >> 8)};
  return SetN2kVictronVreg(msg, destination, reg, data, sizeof(data));
}
}  // namespace

bool SetN2kVictronVreg(tN2kMsg &msg, uint8_t destination, uint16_t registerId,
                      const uint8_t *data, size_t length, bool fastPacket) {
  if (destination == 254 || registerId == 0 || length > (fastPacket ? 219U : 4U) ||
      (length != 0 && data == nullptr)) return false;
  // Copy first so callers may safely reuse data from the output message.
  uint8_t payload[219];
  if (length != 0) std::memcpy(payload, data, length);
  start(msg, fastPacket ? N2kVictronFastPacketPGN : N2kVictronSingleFramePGN, 7, destination);
  msg.Add2ByteUInt(0x9966);
  msg.Add2ByteUInt(registerId);
  for (size_t i = 0; i < length; ++i) msg.AddByte(payload[i]);
  if (!fastPacket) {
    while (msg.DataLen < 8) msg.AddByte(0);
  }
  return true;
}

bool ParseN2kVictronVreg(const tN2kMsg &msg, N2kVictronVreg &out) {
  const bool fast = msg.PGN == N2kVictronFastPacketPGN;
  if ((!fast && msg.PGN != N2kVictronSingleFramePGN) || msg.DataLen < 4 ||
      msg.DataLen > (fast ? 223 : 8)) return false;
  int index = 0;
  if (msg.Get2ByteUInt(index) != 0x9966) return false;
  N2kVictronVreg result;
  result.registerId = msg.Get2ByteUInt(index);
  if (result.registerId == 0) return false;
  result.dataLength = static_cast<size_t>(msg.DataLen - 4);
  std::memcpy(result.data, msg.Data + 4, result.dataLength);
  out = result;
  return true;
}

bool SetN2kVictronRegisterRequest(tN2kMsg &msg, uint8_t destination, uint16_t registerId, uint16_t mask) {
  return setVregPair(msg, destination, 0x0001, registerId, mask);
}
bool ParseN2kVictronRegisterRequest(const tN2kMsg &msg, uint16_t &registerId, uint16_t &mask) {
  return vregPair(msg, 0x0001, registerId, mask);
}
bool SetN2kVictronRegisterAck(tN2kMsg &msg, uint8_t destination, uint16_t registerId, uint16_t ackCode) {
  return setVregPair(msg, destination, 0x0002, registerId, ackCode);
}
bool ParseN2kVictronRegisterAck(const tN2kMsg &msg, uint16_t &registerId, uint16_t &ackCode) {
  return vregPair(msg, 0x0002, registerId, ackCode);
}
bool SetN2kVictronRegisterValue(tN2kMsg &msg, uint8_t destination, uint16_t registerId, uint32_t value) {
  if (registerId <= 2) return false;
  return setVregPair(msg, destination, registerId, static_cast<uint16_t>(value),
                     static_cast<uint16_t>(value >> 16));
}
bool ParseN2kVictronRegisterValue(const tN2kMsg &msg, uint16_t &registerId, uint32_t &value) {
  N2kVictronVreg result;
  if (!ParseN2kVictronVreg(msg, result) || result.registerId <= 2 ||
      result.dataLength == 0 || result.dataLength > 4) return false;
  uint32_t raw = 0;
  for (size_t i = 0; i < result.dataLength; ++i) raw |= uint32_t(result.data[i]) << (8 * i);
  registerId = result.registerId;
  value = raw;
  return true;
}
bool SetN2kVictronDeviceMode(tN2kMsg &msg, uint8_t destination, N2kVictronMode mode) {
  const auto value = static_cast<uint8_t>(mode);
  if (destination > 253 || value < 1 || value > 5) return false;
  return SetN2kVictronRegisterValue(msg, destination, 0x0200, value);
}
bool SetN2kVictronRemoteControl(tN2kMsg &msg, uint8_t destination, uint32_t features) {
  if (destination > 253) return false;
  return SetN2kVictronRegisterValue(msg, destination, 0x0202, features);
}
bool SetN2kVictronCurrentLimit(tN2kMsg &msg, uint8_t destination, double amps, uint8_t input) {
  if (destination > 253 || (input != 1 && input != 2) || N2kIsNA(amps) ||
      !number(amps, 0, 6553.4)) return false;
  return SetN2kVictronRegisterValue(msg, destination, input == 1 ? 0x0214 : 0x0224,
                                   static_cast<uint32_t>(std::round(amps * 10)));
}

bool SetN2kUtilityAcBasic(tN2kMsg &msg, N2kUtilityAcPGN pgn, const N2kAcBasic &data) {
  const auto id = static_cast<unsigned long>(pgn);
  if (!basicPgn(id) || !number(data.lineLineVoltage, 0, 65532) ||
      !number(data.lineNeutralVoltage, 0, 65532) || !number(data.frequency, 0, 511.96875) ||
      !number(data.current, 0, 65532)) return false;
  start(msg, id, 6);
  addUnsigned(msg, data.lineLineVoltage, 1, 0);
  addUnsigned(msg, data.lineNeutralVoltage, 1, 0);
  addUnsigned(msg, data.frequency, 1.0 / 128, 0);
  addUnsigned(msg, data.current, 1, 0);
  return true;
}
bool ParseN2kUtilityAcBasic(const tN2kMsg &msg, N2kAcBasic &out) {
  if (!basicPgn(msg.PGN) || msg.DataLen != 8) return false;
  int index = 0;
  N2kAcBasic data;
  data.lineLineVoltage = getUnsigned(msg, index, 1, 0);
  data.lineNeutralVoltage = getUnsigned(msg, index, 1, 0);
  data.frequency = getUnsigned(msg, index, 1.0 / 128, 0);
  data.current = getUnsigned(msg, index, 1, 0);
  out = data;
  return true;
}
bool SetN2kUtilityAcPower(tN2kMsg &msg, N2kUtilityAcPGN pgn, const N2kAcPower &data) {
  const auto id = static_cast<unsigned long>(pgn);
  if (!powerPgn(id) || !number(data.realPower, -2000000000.0, 2294967292.0) ||
      !number(data.apparentPower, -2000000000.0, 2294967292.0)) return false;
  start(msg, id, 6);
  addUnsigned(msg, data.realPower, 1, -2000000000.0, true);
  addUnsigned(msg, data.apparentPower, 1, -2000000000.0, true);
  return true;
}
bool ParseN2kUtilityAcPower(const tN2kMsg &msg, N2kAcPower &out) {
  if (!powerPgn(msg.PGN) || msg.DataLen != 8) return false;
  int index = 0;
  N2kAcPower data;
  data.realPower = getUnsigned(msg, index, 1, -2000000000.0, true);
  data.apparentPower = getUnsigned(msg, index, 1, -2000000000.0, true);
  out = data;
  return true;
}
bool SetN2kUtilityAcReactivePower(tN2kMsg &msg, N2kUtilityAcPGN pgn, const N2kAcReactivePower &data) {
  const auto id = static_cast<unsigned long>(pgn);
  if (!reactivePgn(id) || !number(data.reactivePower, -2000000000.0, 2294967292.0) ||
      !number(data.powerFactor, -1, 1) || static_cast<uint8_t>(data.lagging) > 3) return false;
  start(msg, id, 6);
  addUnsigned(msg, data.reactivePower, 1, -2000000000.0, true);
  addUnsigned(msg, data.powerFactor, 1.0 / 16384, -1);
  msg.Add2ByteUInt(0xfffc | static_cast<uint8_t>(data.lagging));
  return true;
}
bool ParseN2kUtilityAcReactivePower(const tN2kMsg &msg, N2kAcReactivePower &out) {
  if (!reactivePgn(msg.PGN) || msg.DataLen != 8) return false;
  int index = 0;
  N2kAcReactivePower data;
  data.reactivePower = getUnsigned(msg, index, 1, -2000000000.0, true);
  data.powerFactor = getUnsigned(msg, index, 1.0 / 16384, -1);
  data.lagging = static_cast<N2kPowerFactorLag>(msg.Get2ByteUInt(index) & 3);
  out = data;
  return true;
}
bool SetN2kUtilityAcEnergy(tN2kMsg &msg, const N2kAcEnergy &data) {
  if (!number(data.exportKwh, 0, 4294967292.0) || !number(data.importKwh, 0, 4294967292.0)) return false;
  start(msg, 65005, 6);
  addUnsigned(msg, data.exportKwh, 1, 0, true);
  addUnsigned(msg, data.importKwh, 1, 0, true);
  return true;
}
bool ParseN2kUtilityAcEnergy(const tN2kMsg &msg, N2kAcEnergy &out) {
  if (msg.PGN != 65005 || msg.DataLen != 8) return false;
  int index = 0;
  N2kAcEnergy data;
  data.exportKwh = getUnsigned(msg, index, 1, 0, true);
  data.importKwh = getUnsigned(msg, index, 1, 0, true);
  out = data;
  return true;
}

bool SetN2kPGN127509(tN2kMsg &msg, const N2kInverterStatus &data) {
  if (static_cast<uint8_t>(data.state) > 15 || static_cast<uint8_t>(data.enabled) > 3) return false;
  start(msg, 127509, 6);
  msg.AddByte(data.instance);
  msg.AddByte(data.acInstance);
  msg.AddByte(data.dcInstance);
  msg.AddByte(0xc0 | (static_cast<uint8_t>(data.enabled) << 4) | static_cast<uint8_t>(data.state));
  return true;
}
bool ParseN2kPGN127509(const tN2kMsg &msg, N2kInverterStatus &out) {
  if (msg.PGN != 127509 || msg.DataLen < 4 || msg.DataLen > 8) return false;
  N2kInverterStatus data;
  int index = 0;
  data.instance = msg.GetByte(index);
  data.acInstance = msg.GetByte(index);
  data.dcInstance = msg.GetByte(index);
  const uint8_t flags = msg.GetByte(index);
  data.state = static_cast<N2kInverterState>(flags & 15);
  data.enabled = static_cast<tN2kOnOff>((flags >> 4) & 3);
  out = data;
  return true;
}

bool SetN2kLegacyAcStatus(tN2kMsg &msg, unsigned long pgn, const N2kLegacyAcStatus &data) {
  const uint8_t quality = data.quality == 0xff ? (pgn == 127503 ? 3 : 7) : data.quality;
  if ((pgn != 127503 && pgn != 127504) || data.line > 3 ||
      quality > (pgn == 127503 ? 3 : 7) ||
      !number(data.voltage, 0, 655.32) || !number(data.current, 0, 6553.2) ||
      !number(data.frequency, 0, 655.32) || !number(data.breakerSize, 0, 6553.2) ||
      !number(data.realPower, 0, 4294967292.0) || !number(data.reactivePower, 0, 4294967292.0) ||
      !number(data.powerFactor, -1, 1)) return false;
  start(msg, pgn, 6);
  msg.AddByte(data.instance);
  msg.AddByte(data.numberOfLines);
  msg.AddByte((pgn == 127503 ? 0xf0 : 0xe0) | (quality << 2) | data.line);
  addUnsigned(msg, data.voltage, 0.01, 0);
  addUnsigned(msg, data.current, 0.1, 0);
  addUnsigned(msg, data.frequency, 0.01, 0);
  addUnsigned(msg, data.breakerSize, 0.1, 0);
  addUnsigned(msg, data.realPower, 1, 0, true);
  addUnsigned(msg, data.reactivePower, 1, 0, true);
  msg.Add1ByteDouble(data.powerFactor, 0.01);
  return true;
}
bool ParseN2kLegacyAcStatus(const tN2kMsg &msg, N2kLegacyAcStatus &out) {
  if ((msg.PGN != 127503 && msg.PGN != 127504) || msg.DataLen != 20) return false;
  int index = 0;
  N2kLegacyAcStatus data;
  data.instance = msg.GetByte(index);
  data.numberOfLines = msg.GetByte(index);
  const uint8_t flags = msg.GetByte(index);
  data.line = flags & 3;
  data.quality = (flags >> 2) & (msg.PGN == 127503 ? 3 : 7);
  data.voltage = getUnsigned(msg, index, 0.01, 0);
  data.current = getUnsigned(msg, index, 0.1, 0);
  data.frequency = getUnsigned(msg, index, 0.01, 0);
  data.breakerSize = getUnsigned(msg, index, 0.1, 0);
  data.realPower = getUnsigned(msg, index, 1, 0, true);
  data.reactivePower = getUnsigned(msg, index, 1, 0, true);
  const uint8_t factor = msg.GetByte(index);
  data.powerFactor = factor >= 0x7d && factor <= 0x7f ? N2kDoubleNA :
                    (factor >= 0x80 ? double(factor) - 256.0 : double(factor)) * 0.01;
  out = data;
  return true;
}

bool SetN2kAcPowerCurrent(tN2kMsg &msg, N2kAcPhase phase, const N2kAcPowerCurrent &data) {
  if (static_cast<uint8_t>(phase) > 2 || !number(data.current, 0, 6553.2) ||
      !number(data.power, -2147483648.0, 2147483644.0)) return false;
  start(msg, 127744 + static_cast<uint8_t>(phase), 6);
  msg.AddByte(data.sid);
  msg.AddByte(data.connection);
  addUnsigned(msg, data.current, 0.1, 0);
  msg.Add4ByteDouble(data.power, 1);
  return true;
}
bool ParseN2kAcPowerCurrent(const tN2kMsg &msg, N2kAcPowerCurrent &out) {
  if (msg.PGN < 127744 || msg.PGN > 127746 || msg.DataLen != 8) return false;
  int index = 0;
  N2kAcPowerCurrent data;
  data.sid = msg.GetByte(index);
  data.connection = msg.GetByte(index);
  data.current = getUnsigned(msg, index, 0.1, 0);
  const uint32_t raw = msg.Get4ByteUInt(index);
  data.power = raw >= 0x7ffffffdUL && raw <= 0x7fffffffUL ? N2kDoubleNA :
               (raw >= 0x80000000UL ? double(raw) - 4294967296.0 : double(raw));
  out = data;
  return true;
}
bool SetN2kAcVoltageFrequency(tN2kMsg &msg, N2kAcPhase phase, const N2kAcVoltageFrequency &data) {
  if (static_cast<uint8_t>(phase) > 2 || !number(data.lineNeutralVoltage, 0, 6553.2) ||
      !number(data.lineLineVoltage, 0, 6553.2) || !number(data.frequency, 0, 6553.2)) return false;
  start(msg, 127747 + static_cast<uint8_t>(phase), 6);
  msg.AddByte(data.sid);
  msg.AddByte(data.connection);
  addUnsigned(msg, data.lineNeutralVoltage, 0.1, 0);
  addUnsigned(msg, data.lineLineVoltage, 0.1, 0);
  addUnsigned(msg, data.frequency, 0.1, 0);
  return true;
}
bool ParseN2kAcVoltageFrequency(const tN2kMsg &msg, N2kAcVoltageFrequency &out) {
  if (msg.PGN < 127747 || msg.PGN > 127749 || msg.DataLen != 8) return false;
  int index = 0;
  N2kAcVoltageFrequency data;
  data.sid = msg.GetByte(index);
  data.connection = msg.GetByte(index);
  data.lineNeutralVoltage = getUnsigned(msg, index, 0.1, 0);
  data.lineLineVoltage = getUnsigned(msg, index, 0.1, 0);
  data.frequency = getUnsigned(msg, index, 0.1, 0);
  out = data;
  return true;
}
