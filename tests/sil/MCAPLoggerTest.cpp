// #include <cassert>
// #include <cstdint>
// #include <gtest/gtest.h>
// #include <FoxgloveServer.hpp>
// #include <MCAPLogger.hpp>

// class MCAPLoggerTest : public testing::Test {

// protected:

//   MCAPLoggerTest() {
//     core::FoxgloveServer::create(TEST_CONFIG_DIR + std::string("/fake_foxglove_params.json"));
//    };
// };


// TEST_F(MCAPLoggerTest, ParamFetch) {
//   ASSERT_EQ(core::FoxgloveServer::instance().get_param<int64_t>("rpm_limit"), 10000);
//   ASSERT_EQ(core::FoxgloveServer::instance().get_param<bool>("enable_logging"), true);
// }
