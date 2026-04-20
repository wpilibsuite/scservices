#pragma once

#include <wpi/util/protobuf/Protobuf.hpp>
#include <stdint.h>
#include <array>
#include <optional>
#include "proto/Motioncore.npb.h"

namespace mc {
struct MotioncoreEncoder {
    int32_t position;
    int16_t velocity;
};

struct RawMotioncoreData {
    uint64_t timestampUs;
    std::array<MotioncoreEncoder, 3> encoders;
    uint8_t pinStates;
};
}  // namespace mc

template <>
struct wpi::util::Protobuf<mc::MotioncoreEncoder> {
    using MessageStruct = motioncore_proto_ProtobufMotioncoreEncoder;
    using InputStream = wpi::util::ProtoInputStream<mc::MotioncoreEncoder>;
    using OutputStream = wpi::util::ProtoOutputStream<mc::MotioncoreEncoder>;
    static std::optional<mc::MotioncoreEncoder> Unpack(InputStream& Stream);
    static bool Pack(OutputStream& Stream, const mc::MotioncoreEncoder& Value);
};

template <>
struct wpi::util::Protobuf<mc::RawMotioncoreData> {
    using MessageStruct = motioncore_proto_ProtobufRawMotioncoreData;
    using InputStream = wpi::util::ProtoInputStream<mc::RawMotioncoreData>;
    using OutputStream = wpi::util::ProtoOutputStream<mc::RawMotioncoreData>;
    static std::optional<mc::RawMotioncoreData> Unpack(InputStream& Stream);
    static bool Pack(OutputStream& Stream, const mc::RawMotioncoreData& Value);
};
