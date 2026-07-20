// DDS loopback probe (no hardware needed): UwbTransmitter -> lo ->
// UwbReceiver in one process. Verifies the full wire contract — topic
// name, PoseStamped_ IDL, QoS compatibility, stamp/position mapping,
// and the set_on_position hook.

#include "uwb/uwb_transmitter.hpp"
#include "uwb/uwb_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

using namespace kist;

static int g_failures = 0;

static void check(const char* name, bool ok) {
    std::printf("%-46s %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++g_failures;
}

int main() {
    std::atomic<int>    hook_calls{0};
    std::atomic<float>  hook_last_x{0.0f};

    auto& reader = UwbReceiver::instance();
    reader.set_on_position([&](const UwbPosition& f) {
        ++hook_calls;
        hook_last_x = f.x;
    });
    if (!reader.start(0, "lo")) {
        std::printf("FAILED to start reader on lo\n");
        return 1;
    }

    UwbTransmitter publisher;
    if (!publisher.start(0, "lo")) {
        std::printf("FAILED to start publisher on lo\n");
        return 1;
    }

    // DDS discovery takes a moment — publish at 20Hz until the reader
    // sees data (or a 5s deadline expires).
    UwbSample sample;
    sample.x = 1.25f;
    sample.y = -2.5f;
    sample.z = 0.9f;
    sample.quality = 77;

    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::seconds(5);
    while (clock::now() < deadline && !reader.uwb_buf.GetData())
        {
        publisher.publish(sample, "map");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    auto fix = reader.uwb_buf.GetData();
    check("fix received over loopback DDS", fix != nullptr);
    if (fix) {
        check("position mapped", fix->x == 1.25f && fix->y == -2.5f && fix->z == 0.9f);
        check("publisher stamped (stamp_ns > 0)", fix->stamp_ns > 0);
    }
    check("set_on_position hook fired", hook_calls > 0 && hook_last_x == 1.25f);

    // second fix with new coordinates lands too (stream, not one-shot)
    sample.x = 3.5f;
    const auto deadline2 = clock::now() + std::chrono::seconds(3);
    while (clock::now() < deadline2) {
        publisher.publish(sample, "map");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto f = reader.uwb_buf.GetData();
        if (f && f->x == 3.5f)
            break;
    }
    auto f2 = reader.uwb_buf.GetData();
    check("subsequent fix received", f2 && f2->x == 3.5f);

    reader.stop();
    std::printf("\n%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
