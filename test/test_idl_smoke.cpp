// idlc smoke test (verify TOGETHER in the container). The whole idlc
// switch hinges on one unverified assumption: does an idlc-generated
// type (CycloneDDS-CXX 0.10.2 backend) plug into the unitree SDK's
// ChannelPublisher<T>? This proves it at compile+link and at runtime
// over lo. If this passes, rewiring the real publishers/receivers is
// mechanical.
//
//   ./test_idl_smoke

#include "kist_camera_frames.hpp"   // idlc-generated (build/idl_gen)

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include <cstdio>
#include <vector>

int main() {
    // 1) generated struct: accessors compile & work
    kist_msgs::CompressedColorFrame f;
    f.width(640);
    f.height(480);
    f.is_keyframe(true);
    f.frame_id("camera_color");
    f.data(std::vector<uint8_t>{1, 2, 3, 4});
    std::printf("struct ok: %ux%u keyframe=%d data=%zu\n",
                f.width(), f.height(), int(f.is_keyframe()), f.data().size());

    // 2) the real question: SDK ChannelPublisher accepts the generated type
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, "lo");
        unitree::robot::ChannelPublisher<kist_msgs::CompressedColorFrame> pub("rt/kist/test/color");
        pub.InitChannel();
        pub.Write(f);
        std::printf("ChannelPublisher<kist_msgs::CompressedColorFrame> OK (typed DDS works)\n");
    } catch (const std::exception& e) {
        std::printf("DDS error: %s\n", e.what());
        return 1;
    }
    return 0;
}
