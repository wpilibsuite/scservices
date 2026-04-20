#include "MotioncoreProto.h"

#include "wpi/util/protobuf/ProtobufCallbacks.hpp"

std::optional<mc::MotioncoreEncoder>
wpi::util::Protobuf<mc::MotioncoreEncoder>::Unpack(InputStream& Stream) {
    motioncore_proto_ProtobufMotioncoreEncoder msg{
        .Position = 0,
        .Velocity = 0,
    };
    if (!Stream.Decode(msg)) {
        return std::nullopt;
    }
    mc::MotioncoreEncoder result;
    result.position = msg.Position;
    result.velocity = msg.Velocity;
    return result;
}

bool wpi::util::Protobuf<mc::MotioncoreEncoder>::Pack(
    OutputStream& Stream, const mc::MotioncoreEncoder& Value) {
    motioncore_proto_ProtobufMotioncoreEncoder msg{
        .Position = Value.position,
        .Velocity = Value.velocity,
    };
    return Stream.Encode(msg);
}

std::optional<mc::RawMotioncoreData>
wpi::util::Protobuf<mc::RawMotioncoreData>::Unpack(InputStream& Stream) {
    wpi::util::UnpackCallback<mc::MotioncoreEncoder> cb[3];
    motioncore_proto_ProtobufRawMotioncoreData msg{
        .Encoder0 = cb[0].Callback(),
        .Encoder1 = cb[1].Callback(),
        .Encoder2 = cb[2].Callback(),
        .Timestamp = 0,
        .PinStates = 0,
    };

    if (!Stream.Decode(msg)) {
        return std::nullopt;
    }

    mc::RawMotioncoreData result;
    result.timestampUs = msg.Timestamp;
    result.pinStates = msg.PinStates;
    for (size_t i = 0; i < 3; i++) {
        const auto& items = cb[i].Items();
        if (items.size() != 1) {
            // We should have exactly 1 item for each encoder, otherwise
            // something went wrong
            return std::nullopt;
        }
        result.encoders[i] = items[0];
    }
    return result;
}

bool wpi::util::Protobuf<mc::RawMotioncoreData>::Pack(
    OutputStream& Stream, const mc::RawMotioncoreData& Value) {
    wpi::util::PackCallback<mc::MotioncoreEncoder> cb[3]{
        wpi::util::PackCallback<mc::MotioncoreEncoder>{&Value.encoders[0]},
        wpi::util::PackCallback<mc::MotioncoreEncoder>{&Value.encoders[1]},
        wpi::util::PackCallback<mc::MotioncoreEncoder>{&Value.encoders[2]},
    };

    motioncore_proto_ProtobufRawMotioncoreData msg{
        .Encoder0 = cb[0].Callback(),
        .Encoder1 = cb[1].Callback(),
        .Encoder2 = cb[2].Callback(),
        .Timestamp = Value.timestampUs,
        .PinStates = Value.pinStates,
    };

    return Stream.Encode(msg);
}
