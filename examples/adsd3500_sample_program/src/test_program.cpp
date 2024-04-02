#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

struct buffer {
    void* start;
    size_t length;
};

void init_mmap(int fd, const char* dev_name, buffer*& buffers, unsigned int& n_buffers) {
    struct v4l2_requestbuffers req;
    CLEAR(req);


    req.count = 4; // Number of buffers
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        if (errno == EINVAL) {
            std::cerr << dev_name << " does not support memory mapping" << std::endl;
            exit(EXIT_FAILURE);
        } else {
            std::cerr << "VIDIOC_REQBUFS failed" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    if (req.count < 2) {
        std::cerr << "Insufficient buffer memory on " << dev_name << std::endl;
        exit(EXIT_FAILURE);
    }

    buffers = new buffer[req.count];
    if (!buffers) {
        std::cerr << "Out of memory" << std::endl;
        exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
            std::cerr << "VIDIOC_QUERYBUF failed" << std::endl;
            exit(EXIT_FAILURE);
        }

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);

        if (buffers[n_buffers].start == MAP_FAILED) {
            std::cerr << "mmap failed" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

void capture_frames(int fd, const char* dev_name, buffer* buffers, unsigned int n_buffers) {
    struct v4l2_buffer buf;
    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    for (;;) {
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            std::cerr << "VIDIOC_DQBUF failed" << std::endl;
            exit(EXIT_FAILURE);
        }

        // Process captured frame here
        // You can access the frame data through buffers[buf.index].start
        std::cout << "Received frame " << buf.index << std::endl;

        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            std::cerr << "VIDIOC_QBUF failed" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}

int main() {
    const char* dev_name = "/dev/video0";
    int fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) {
        std::cerr << "Cannot open " << dev_name << std::endl;
        return EXIT_FAILURE;
    }

    buffer* buffers;
    unsigned int n_buffers;

    init_mmap(fd, dev_name, buffers, n_buffers);
    capture_frames(fd, dev_name, buffers, n_buffers);

    close(fd);
    return EXIT_SUCCESS;
}
