#include "UARTComms.hpp"

#include <gtest/gtest.h>
#include <google/protobuf/descriptor.pb.h>

#include <algorithm>
#include <limits>
#include <random>
#include <stdexcept>

namespace comms {

struct UARTCodecTestAccess {
    using Driver = UARTComms;

    static auto encode_message(const Driver& driver, const Driver::Message& message) {
        return driver._encode_message(message);
    }

    static auto decode_message(const Driver& driver, const std::vector<uint8_t>& frame) {
        return driver._decode_message(frame);
    }

    static const std::string& topic(const Driver& driver) {
        return driver._config.receive_topic;
    }

    static auto encode(const std::vector<uint8_t>& payload, std::size_t limit) {
        return Driver::_encode_uart_frame(payload, limit);
    }

    static auto decode(const std::vector<uint8_t>& frame, std::size_t limit) {
        return Driver::_decode_uart_frame(frame, limit);
    }
};

} // namespace comms

using comms::UARTCodecTestAccess;

TEST(UARTCodec, MatchesKnownCRCVectorAndWireFormat) {
    const std::vector<uint8_t> payload{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    // CRC-16/CCITT-FALSE("123456789") = 0x29B1, sent low byte first.
    const std::vector<uint8_t> expected{12, '1', '2', '3', '4', '5', '6', '7', '8', '9',
                                        0xB1, 0x29, 0};
    const auto frame = UARTCodecTestAccess::encode(payload, 4096);
    EXPECT_EQ(frame, expected);

    const auto decoded = UARTCodecTestAccess::decode({expected.begin(), expected.end() - 1}, 4096);
    ASSERT_TRUE(decoded);
    EXPECT_EQ(*decoded, payload);
}

TEST(UARTCodec, SupportsEmptyPayload) {
    const auto frame = UARTCodecTestAccess::encode({}, 0);
    EXPECT_EQ(frame, (std::vector<uint8_t>{3, 0xFF, 0xFF, 0}));
    const auto decoded = UARTCodecTestAccess::decode({3, 0xFF, 0xFF}, 0);
    ASSERT_TRUE(decoded);
    EXPECT_TRUE(decoded->empty());
}

TEST(UARTCodec, RoundTripsZerosAndCOBSBlockBoundaries) {
    std::mt19937 random(42);
    for (std::size_t size : {1, 2, 252, 253, 254, 255, 256, 508, 509, 4096}) {
        for (int pattern = 0; pattern < 3; ++pattern) {
            std::vector<uint8_t> payload(size);
            for (auto& byte : payload) {
                byte = pattern == 0 ? 0 : pattern == 1 ? 0x55 : random() & 0xFF;
            }
            auto frame = UARTCodecTestAccess::encode(payload, size);
            ASSERT_FALSE(frame.empty());
            ASSERT_EQ(frame.back(), 0);
            EXPECT_EQ(std::count(frame.begin(), frame.end() - 1, 0), 0);
            frame.pop_back();
            const auto decoded = UARTCodecTestAccess::decode(frame, size);
            ASSERT_TRUE(decoded) << "payload size " << size << ", pattern " << pattern;
            EXPECT_EQ(*decoded, payload);
        }
    }
}

TEST(UARTCodec, RejectsCorruptionAndMalformedCOBS) {
    EXPECT_FALSE(UARTCodecTestAccess::decode({}, 4096));
    EXPECT_FALSE(UARTCodecTestAccess::decode({0}, 4096));
    EXPECT_FALSE(UARTCodecTestAccess::decode({4, 1}, 4096)); // Truncated block.
    EXPECT_FALSE(UARTCodecTestAccess::decode({2, 0}, 4096)); // Zero inside block.
    EXPECT_FALSE(UARTCodecTestAccess::decode({1}, 4096)); // No CRC.

    auto frame = UARTCodecTestAccess::encode({1, 2, 3}, 4096);
    EXPECT_FALSE(UARTCodecTestAccess::decode(frame, 4096)); // Delimiter must be removed.
    frame.pop_back();
    frame[1] ^= 0x10;
    EXPECT_FALSE(UARTCodecTestAccess::decode(frame, 4096)); // CRC mismatch.
}

TEST(UARTCodec, EnforcesPayloadLimitsOnBothPaths) {
    EXPECT_THROW(UARTCodecTestAccess::encode({1, 2, 3}, 2), std::length_error);
    const auto unsupported_limit = static_cast<std::size_t>(std::numeric_limits<int>::max()) + 1;
    EXPECT_THROW(UARTCodecTestAccess::encode({}, unsupported_limit), std::invalid_argument);
    auto frame = UARTCodecTestAccess::encode({1, 2, 3}, 3);
    frame.pop_back();
    EXPECT_TRUE(UARTCodecTestAccess::decode(frame, 3));
    EXPECT_FALSE(UARTCodecTestAccess::decode(frame, 2));
    EXPECT_FALSE(UARTCodecTestAccess::decode(std::vector<uint8_t>(5000, 1), 4096));
}

TEST(UARTCodec, RoundTripsProtobufUsingOwnedReceivePrototype) {
    // The driver's prototype must survive destruction of the caller's object.
    auto driver = [] {
        google::protobuf::FileDescriptorProto prototype;
        prototype.set_name("prototype fields must not appear in decoded messages");
        return std::make_unique<comms::UARTComms>(
            comms::UARTComms::Config{}, prototype, [](auto) {});
    }();

    google::protobuf::FileDescriptorProto outgoing;
    outgoing.set_name("controller.proto");
    outgoing.add_dependency("status.proto");
    auto frame = UARTCodecTestAccess::encode_message(*driver, outgoing);
    frame.pop_back();

    auto first = UARTCodecTestAccess::decode_message(*driver, frame);
    auto second = UARTCodecTestAccess::decode_message(*driver, frame);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first.get(), second.get());

    auto typed = std::dynamic_pointer_cast<google::protobuf::FileDescriptorProto>(first);
    ASSERT_TRUE(typed);
    EXPECT_EQ(typed->name(), outgoing.name());
    ASSERT_EQ(typed->dependency_size(), 1);
    EXPECT_EQ(typed->dependency(0), "status.proto");

    const auto empty_frame = UARTCodecTestAccess::encode({}, 4096);
    const auto empty_message = UARTCodecTestAccess::decode_message(
        *driver, {empty_frame.begin(), empty_frame.end() - 1});
    ASSERT_TRUE(empty_message);
    EXPECT_EQ(empty_message->ByteSizeLong(), 0);
}

TEST(UARTCodec, RejectsInvalidProtobufEvenWithValidFraming) {
    comms::UARTComms driver({}, google::protobuf::FileDescriptorProto{}, [](auto) {});
    auto frame = UARTCodecTestAccess::encode({0x0A, 0x05, 'x'}, 4096);
    frame.pop_back();
    EXPECT_FALSE(UARTCodecTestAccess::decode_message(driver, frame));
    EXPECT_FALSE(UARTCodecTestAccess::decode_message(driver, {0}));
}

TEST(UARTCodec, EnforcesPayloadLimitForProtobufMessages) {
    comms::UARTComms::Config config;
    config.max_payload_bytes = 2;
    comms::UARTComms driver(config, google::protobuf::FileDescriptorProto{}, [](auto) {});
    google::protobuf::FileDescriptorProto message;
    message.set_name("too large");
    try {
        UARTCodecTestAccess::encode_message(driver, message);
        FAIL() << "Expected an oversized-payload exception";
    } catch (const std::length_error& error) {
        const std::string description = error.what();
        EXPECT_NE(description.find(message.GetTypeName()), std::string::npos);
        EXPECT_NE(description.find(std::to_string(message.ByteSizeLong()) + " bytes"), std::string::npos);
        EXPECT_NE(description.find("maximum is 2"), std::string::npos);
    }

    auto frame = UARTCodecTestAccess::encode({1, 2, 3}, 4096);
    frame.pop_back();
    EXPECT_FALSE(UARTCodecTestAccess::decode_message(driver, frame));
}

TEST(UARTCodec, RejectsProtobufWithMissingRequiredFields) {
    comms::UARTComms driver({}, google::protobuf::FileDescriptorProto{}, [](auto) {});
    google::protobuf::UninterpretedOption::NamePart incomplete;
    ASSERT_FALSE(incomplete.IsInitialized());
    try {
        UARTCodecTestAccess::encode_message(driver, incomplete);
        FAIL() << "Expected a missing-required-fields exception";
    } catch (const std::invalid_argument& error) {
        const std::string description = error.what();
        EXPECT_NE(description.find(incomplete.GetTypeName()), std::string::npos);
        EXPECT_NE(description.find("missing required fields"), std::string::npos);
        EXPECT_NE(description.find("name_part"), std::string::npos);
    }
}

TEST(UARTCodec, GeneratesReceiveTopicAndPreservesCustomTopic) {
    comms::UARTComms generated({}, google::protobuf::FileDescriptorProto{}, [](auto) {});
    EXPECT_EQ(UARTCodecTestAccess::topic(generated), "uart/filedescriptorproto/rx");

    comms::UARTComms::Config config;
    config.receive_topic = "uart/front/rx";
    comms::UARTComms custom(config, google::protobuf::DescriptorProto{}, [](auto) {});
    EXPECT_EQ(UARTCodecTestAccess::topic(custom), "uart/front/rx");
}

TEST(UARTCodec, RejectsEmptyHandlerAndUnsupportedPayloadLimit) {
    EXPECT_THROW((comms::UARTComms({}, google::protobuf::FileDescriptorProto{}, {})),
                 std::invalid_argument);
    comms::UARTComms::Config config;
    config.max_payload_bytes = static_cast<std::size_t>(std::numeric_limits<int>::max()) + 1;
    EXPECT_THROW((comms::UARTComms(config, google::protobuf::FileDescriptorProto{}, [](auto) {})),
                 std::invalid_argument);
}
