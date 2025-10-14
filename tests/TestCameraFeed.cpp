#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <sys/mman.h>      // for mmap
#include <fcntl.h>
#include <unistd.h>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

using namespace libcamera;

int main() {
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    if (cm->cameras().empty()) {
        std::cerr << "No camera detected!\n";
        return -1;
    }

    auto camera = cm->get(cm->cameras()[0]->id());
    camera->acquire();

    auto config = camera->generateConfiguration({ StreamRole::Viewfinder });
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.size.width = 640;
    streamConfig.size.height = 480;

    config->validate();
    camera->configure(config.get());

    FrameBufferAllocator allocator(camera);
    if (allocator.allocate(streamConfig.stream()) < 0) {
        std::cerr << "Failed to allocate buffers\n";
        return -1;
    }

    const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator.buffers(streamConfig.stream());
    if (buffers.empty()) {
        std::cerr << "No buffers allocated\n";
        return -1;
    }

    std::vector<std::unique_ptr<Request>> requests;
    for (auto &fb : buffers) {
        auto request = camera->createRequest();
        if (!request) {
            std::cerr << "Failed to create request\n";
            return -1;
        }
        if (request->addBuffer(streamConfig.stream(), fb.get()) < 0) {
            std::cerr << "Failed to add buffer to request\n";
            return -1;
        }
        requests.push_back(std::move(request));
    }

    // --- start camera and queue first request
    camera->start();
    camera->queueRequest(requests[0].get());

    std::unique_ptr<Request> completed;
while (true) {
    completed = camera->getCompletedRequests().front();
    if (completed)
        break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

    // --- cleanup
    camera->stop();
    camera->release();
    cm->stop();

    return 0;
}
