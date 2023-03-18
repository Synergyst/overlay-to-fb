// overlay-to-fb - Overlay images onto HDMI input and then output to frame buffer via HDMI
#include <iostream>
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <omp.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <linux/fb.h>
#include "../convert-png-to-rgb24-with-alpha.hpp"

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#ifndef V4L2_PIX_FMT_H264
#define V4L2_PIX_FMT_H264     v4l2_fourcc('H', '2', '6', '4') /* H264 with start codes */
#endif

bool isFirstFrame = true;

enum io_method {
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer {
        void   *start;
        size_t length;
};

static char            *dev_name;
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
struct buffer          *buffers;
static unsigned int n_buffers;
static int out_buf;
static int force_format = 3;
static int frame_count = 0;
static int frame_number = 0;

// The width and height of the input video and any downscaled output video
static const int startingWidth = 1280;
static const int startingHeight = 720;
static const int scaledOutWidth = 640;
static const int scaledOutHeight = 360;
static int croppedWidth = 0;
static int croppedHeight = 0;
// Crop size matrix (scale up or down as needed)
static int cropMatrix[2][4] = { {11, 4, 4, 2}, {1, 1, 1, 1} };
// The maximum value for the Sobel operator
static const int maxSobel = 4 * 255;
// The Sobel operator as a 3x3 matrix
static const int sobelX[3][3] = { {-1, 0, 1}, {-2, 0, 2}, {-1, 0, 1} };
static const int sobelY[3][3] = { {-1, -2, -1}, {0, 0, 0}, {1, 2, 1} };

static unsigned char* outputFrame = new unsigned char[startingWidth * startingHeight * 2];
static unsigned char* outputFrame2 = new unsigned char[startingWidth * startingHeight * 2];
static unsigned char* outputFrameRGB24 = new unsigned char[startingWidth * startingHeight * 3];
static unsigned char* outputFrameGreyscale = new unsigned char[startingWidth * startingHeight];
static unsigned char* outputFrameGreyscale1 = new unsigned char[startingWidth * startingHeight];
static unsigned char* outputFrameGreyscale2 = new unsigned char[startingWidth * startingHeight];
static unsigned char* outputFrameGreyscale3 = new unsigned char[startingWidth * startingHeight];
static unsigned char* outputFrameGreyscale4 = new unsigned char[startingWidth * startingHeight];
static unsigned char* outputFrameGreyscale5 = new unsigned char[startingWidth * startingHeight];
static unsigned char* redVals = new unsigned char[startingWidth * startingHeight];
static unsigned char* greenVals = new unsigned char[startingWidth * startingHeight];
static unsigned char* blueVals = new unsigned char[startingWidth * startingHeight];
static unsigned char* outputFrameScaledR = new unsigned char[scaledOutWidth * scaledOutHeight];
static unsigned char* outputFrameScaledG = new unsigned char[scaledOutWidth * scaledOutHeight];
static unsigned char* outputFrameScaledB = new unsigned char[scaledOutWidth * scaledOutHeight];

Image overlayImage;

string overlayFilename = "./pen15.png";
std::vector<unsigned char> merged_data(startingWidth * startingHeight * 3);
constexpr int num_threads = 4;

static void errno_exit(const char *s) {
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg) {
        int r;
        do {
          r = ioctl(fh, request, arg);
        } while (-1 == r && EINTR == errno);
        return r;
}

//unsigned int w = 1280, h = 720;
unsigned int w = 640, h = 480;
unsigned char red, green, blue;

static void rgb24_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 3;
      // Extract the red, green, and blue components from the RGB pixel
      unsigned char r = input[offset];
      unsigned char g = input[offset + 1];
      unsigned char b = input[offset + 2];
      // Calculate the greyscale value using the formula:
      // greyscale = 0.299 * red + 0.587 * green + 0.114 * blue
      unsigned char greyscale = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);
      // Set the greyscale value as the intensity of the output pixel
      output[y * width + x] = greyscale;
    }
  }
}

static void rgb24_to_yuyv(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 3;
      // Extract the red, green, and blue components from the RGB pixel
      unsigned char r = input[offset];
      unsigned char g = input[offset + 1];
      unsigned char b = input[offset + 2];
      // Calculate the Y, U, and V components using the following formulas:
      // Y = 0.299 * red + 0.587 * green + 0.114 * blue
      // U = -0.147 * red - 0.289 * green + 0.436 * blue + 128
      // V = 0.615 * red - 0.515 * green - 0.100 * blue + 128
      unsigned char y = static_cast<unsigned char>(0.299 * r + 0.587 * g + 0.114 * b);
      unsigned char u = static_cast<unsigned char>(-0.147 * r - 0.289 * g + 0.436 * b + 128);
      unsigned char v = static_cast<unsigned char>(0.615 * r - 0.515 * g - 0.100 * b + 128);
      // Pack the Y, U, and V components into a YUYV pixel
      output[offset] = y;
      output[offset + 1] = u;
      output[offset + 2] = y;
      output[offset + 3] = v;
    }
  }
}

static void uyvy_to_greyscale(unsigned char* input, unsigned char* output, int width, int height) {
  // Iterate over each pixel in the input image
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      // Calculate the offset into the input buffer for the current pixel
      int offset = (y * width + x) * 2;
      // Extract the Y component from the UYVY pixel
      output[y * width + x] = input[offset];
    }
  }
}

static void yuyv_to_greyscale(const unsigned char* yuyv, unsigned char* grey, int width, int height) {
#pragma omp parallel for
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      // YUYV format stores chroma (Cb and Cr) values interleaved with the luma (Y) values.
      // So, we need to skip every other pixel.
      int y_index = index * 2;
      grey[index] = yuyv[y_index];
    }
  }
}

static void frame_to_stdout(unsigned char* input, int size) {
  int status = write(1, input, size);
  if (status == -1)
    perror("write");
}

void overlayRGB24Frames(void *p1, void *p2, int width, int height, int numThreads = std::thread::hardware_concurrency()) {
    unsigned char *frame1 = static_cast<unsigned char *>(p1);
    unsigned char *frame2 = static_cast<unsigned char *>(p2);
    const int numPixels = width * height;
    const int numBytes = numPixels * 3;
    std::vector<std::thread> threads(numThreads);
    for (int t = 0; t < numThreads; ++t) {
        const int start = (t * numBytes) / numThreads;
        const int end = ((t + 1) * numBytes) / numThreads;
        threads[t] = std::thread([=] {
            for (int i = start; i < end; i += 3) {
                const int r1 = frame1[i];
                const int g1 = frame1[i + 1];
                const int b1 = frame1[i + 2];
                const int r2 = frame2[i];
                const int g2 = frame2[i + 1];
                const int b2 = frame2[i + 2];
                const int alpha = r2 + g2 + b2;
                if (alpha > 0) {
                    const int r = ((r1 * (255 - alpha)) + (r2 * alpha)) / 255;
                    const int g = ((g1 * (255 - alpha)) + (g2 * alpha)) / 255;
                    const int b = ((b1 * (255 - alpha)) + (b2 * alpha)) / 255;
                    frame1[i] = static_cast<unsigned char>(r);
                    frame1[i + 1] = static_cast<unsigned char>(g);
                    frame1[i + 2] = static_cast<unsigned char>(b);
                }
            }
        });
    }
    for (auto &t : threads) {
        t.join();
    }
}

void writeFrameToFramebuffer(const unsigned char* frameData) {
    int fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    struct fb_var_screeninfo vinfo;
    ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo);
    if (vinfo.bits_per_pixel != 16 || vinfo.xres != 1280 || vinfo.yres != 720) {
        perror("Error: framebuffer does not accept RGB24 frames with 1280x720 resolution");
        exit(1);
    }
    long int screensize = vinfo.xres * vinfo.yres * 2;
    /*char* fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if ((int)fbmem == -1) {
        perror("Error: failed to mmap framebuffer device to memory");
        exit(1);
    }*/
    char* fbmem = (char*)mmap(0, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if (fbmem == MAP_FAILED) {
      perror("Error: failed to mmap framebuffer device to memory");
      exit(1);
    }
    char* currentPixel = fbmem;
    for (int y = 0; y < vinfo.yres; y++) {
        for (int x = 0; x < vinfo.xres; x++) {
            int pixelOffset = y * vinfo.xres * 3 + x * 3;
            unsigned char r = frameData[pixelOffset];
            unsigned char g = frameData[pixelOffset + 1];
            unsigned char b = frameData[pixelOffset + 2];
            unsigned short pixel = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
            *(unsigned short*)currentPixel = pixel;
            currentPixel += 2;
        }
    }
    munmap(fbmem, screensize);
    close(fbfd);
}

/*void overlayRGBA32OnRGB24(unsigned char* rgb24, const unsigned char* rgba32, int width, int height) {
    const int numPixels = width * height;
    const int numBytesRGB24 = numPixels * 3;
    const int numBytesRGBA32 = numPixels * 4;
    for (int i = 0; i < numPixels; ++i) {
        const int r1 = rgb24[i * 3];
        const int g1 = rgb24[i * 3 + 1];
        const int b1 = rgb24[i * 3 + 2];
        const int r2 = rgba32[i * 4];
        const int g2 = rgba32[i * 4 + 1];
        const int b2 = rgba32[i * 4 + 2];
        const int alpha = rgba32[i * 4 + 3];
        if (alpha > 0) {
            const float alpha_ratio = alpha / 255.0f;
            const int r = static_cast<int>(r1 * (1 - alpha_ratio) + r2 * alpha_ratio);
            const int g = static_cast<int>(g1 * (1 - alpha_ratio) + g2 * alpha_ratio);
            const int b = static_cast<int>(b1 * (1 - alpha_ratio) + b2 * alpha_ratio);
            rgb24[i * 3] = static_cast<unsigned char>(r);
            rgb24[i * 3 + 1] = static_cast<unsigned char>(g);
            rgb24[i * 3 + 2] = static_cast<unsigned char>(b);
        }
    }
}*/

void overlayRGBA32OnRGB24(unsigned char* rgb24, const unsigned char* rgba32, int width, int height, int numThreads = std::thread::hardware_concurrency()) {
    const int numPixels = width * height;
    const int numBytesRGB24 = numPixels * 3;
    const int numBytesRGBA32 = numPixels * 4;
    std::vector<std::thread> threads(numThreads);
    for (int t = 0; t < numThreads; ++t) {
        const int start = (t * numPixels) / numThreads;
        const int end = ((t + 1) * numPixels) / numThreads;
        threads[t] = std::thread([=] {
            for (int i = start; i < end; ++i) {
                const int r1 = rgb24[i * 3];
                const int g1 = rgb24[i * 3 + 1];
                const int b1 = rgb24[i * 3 + 2];
                const int r2 = rgba32[i * 4];
                const int g2 = rgba32[i * 4 + 1];
                const int b2 = rgba32[i * 4 + 2];
                const int alpha = rgba32[i * 4 + 3];
                if (alpha > 0) {
                    const float alpha_ratio = alpha / 255.0f;
                    const int r = static_cast<int>(r1 * (1 - alpha_ratio) + r2 * alpha_ratio);
                    const int g = static_cast<int>(g1 * (1 - alpha_ratio) + g2 * alpha_ratio);
                    const int b = static_cast<int>(b1 * (1 - alpha_ratio) + b2 * alpha_ratio);
                    rgb24[i * 3] = static_cast<unsigned char>(r);
                    rgb24[i * 3 + 1] = static_cast<unsigned char>(g);
                    rgb24[i * 3 + 2] = static_cast<unsigned char>(b);
                }
            }
        });
    }
    for (auto& t : threads) {
        t.join();
    }
}

void convertRGBtoBGR(unsigned char* rgb24Data, unsigned char* bgr24Data, int width, int height) {
    int numPixels = width * height;
    int rgbIndex = 0;
    int bgrIndex = 0;
    for (int i = 0; i < numPixels; i++) {
        bgr24Data[bgrIndex++] = rgb24Data[rgbIndex + 2];  // B value
        bgr24Data[bgrIndex++] = rgb24Data[rgbIndex + 1];  // G value
        bgr24Data[bgrIndex++] = rgb24Data[rgbIndex];      // R value
        rgbIndex += 3;
    }
}

static void process_image(const void *p, int size) {
  unsigned char* preP = (unsigned char*)p;
  std::memcpy(merged_data.data(), preP, 1280 * 720 * 3);
  overlayRGBA32OnRGB24(merged_data.data(), overlayImage.data.data(), startingWidth, startingHeight, num_threads);
  //convertRGBtoBGR(merged_data.data(), merged_data.data(), startingWidth, startingHeight);
  writeFrameToFramebuffer(merged_data.data());
}

static int read_frame(void) {
        struct v4l2_buffer buf;
        unsigned int i;
        switch (io) {
        case IO_METHOD_READ:
                if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;
                        case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                        default:
                                errno_exit("read");
                        }
                }
                process_image(buffers[0].start, buffers[0].length);
                break;
        case IO_METHOD_MMAP:
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;
                        case EIO:
                        /* Could ignore EIO, see spec. */
                        /* fall through */
                        default:
                                errno_exit("VIDIOC_DQBUF");
                        }
                }
                assert(buf.index < n_buffers);
                process_image(buffers[buf.index].start, buf.bytesused);
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
                break;
        case IO_METHOD_USERPTR:
                CLEAR(buf);
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_USERPTR;
                if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
                        switch (errno) {
                        case EAGAIN:
                                return 0;
                        case EIO:
                        /* Could ignore EIO, see spec. */
                        /* fall through */
                        default:
                                errno_exit("VIDIOC_DQBUF");
                        }
                }
                for (i = 0; i < n_buffers; ++i)
                        if (buf.m.userptr == (unsigned long)buffers[i].start
                            && buf.length == buffers[i].length)
                                break;
                assert(i < n_buffers);
                process_image((void *)buf.m.userptr, buf.bytesused);
                if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                        errno_exit("VIDIOC_QBUF");
                break;
        }

        return 1;
}

static void mainloop(void) {
        unsigned int count;
        unsigned int loopIsInfinite = 0;
        if (frame_count == 0) loopIsInfinite = 1; //infinite loop
        count = frame_count;
        while ((count-- > 0) || loopIsInfinite) {
                for (;; ) {
                        fd_set fds;
                        struct timeval tv;
                        int r;
                        FD_ZERO(&fds);
                        FD_SET(fd, &fds);
                        /* Timeout. */
                        tv.tv_sec = 2;
                        tv.tv_usec = 0;
                        r = select(fd + 1, &fds, NULL, NULL, &tv);
                        if (-1 == r) {
                                if (EINTR == errno)
                                        continue;
                                errno_exit("select");
                        }
                        if (0 == r) {
                                fprintf(stderr, "select timeout\n");
                                exit(EXIT_FAILURE);
                        }
                        if (read_frame())
                                break;
                        /* EAGAIN - continue select loop. */
                }
        }
}

static void stop_capturing(void) {
        enum v4l2_buf_type type;
        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

static void start_capturing(void) {
        unsigned int i;
        enum v4l2_buf_type type;
        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;
        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;
                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;
                        buf.index = i;
                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;
                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;
                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

static void uninit_device(void) {
        unsigned int i;
        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;
        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;
        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }
        free(buffers);
}

static void init_read(unsigned int buffer_size) {
        buffers = (buffer*)calloc(1, sizeof(*buffers));
        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);
        if (!buffers[0].start) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void) {
        struct v4l2_requestbuffers req;
        CLEAR(req);
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }
        if (req.count < 2) {
                fprintf(stderr, "Insufficient buffer memory on %s\n",
                        dev_name);
                exit(EXIT_FAILURE);
        }
        buffers = (buffer*)calloc(req.count, sizeof(*buffers));
        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;
                CLEAR(buf);
                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;
                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");
                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                             buf.length,
                             PROT_READ | PROT_WRITE /* required */,
                             MAP_SHARED /* recommended */,
                             fd, buf.m.offset);
                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size) {
        struct v4l2_requestbuffers req;
        CLEAR(req);
        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;
        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }
        buffers = (buffer*)calloc(4, sizeof(*buffers));
        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);
                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

static void init_device(void) {
        struct v4l2_capability cap;
        struct v4l2_cropcap cropcap;
        struct v4l2_crop crop;
        struct v4l2_format fmt;
        unsigned int min;
        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s is no V4L2 device\n",
                                dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_QUERYCAP");
                }
        }
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
                fprintf(stderr, "%s is no video capture device\n",
                        dev_name);
                exit(EXIT_FAILURE);
        }
        switch (io) {
        case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                        fprintf(stderr, "%s does not support read i/o\n",
                                dev_name);
                        exit(EXIT_FAILURE);
                }
                break;
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                        fprintf(stderr, "%s does not support streaming i/o\n",
                                dev_name);
                        exit(EXIT_FAILURE);
                }
                break;
        }
        /* Select video input, video standard and tune here. */
        CLEAR(cropcap);
        cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
                crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                crop.c = cropcap.defrect; /* reset to default */
                if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
                        switch (errno) {
                        case EINVAL:
                                /* Cropping not supported. */
                                break;
                        default:
                                /* Errors ignored. */
                                break;
                        }
                }
        } else {
                /* Errors ignored. */
        }
        CLEAR(fmt);
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fprintf(stderr, "Force Format %d\n", force_format);
        if (force_format) {
                /*if (force_format==3) {
                        fmt.fmt.pix.width       = 640;
                        fmt.fmt.pix.height      = 480;
                        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
                        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
                        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;*/
                if (force_format==3) {
                        fmt.fmt.pix.width       = 1280;
                        fmt.fmt.pix.height      = 720;
                        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
                        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
                        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
                } else if (force_format==2) {
                        fmt.fmt.pix.width       = 640;
                        fmt.fmt.pix.height      = 480;
                        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
                        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
                        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
                } else if (force_format==1) {
                        fmt.fmt.pix.width       = 640;
                        fmt.fmt.pix.height      = 480;
                        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
                        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
                        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
                        //fmt.fmt.pix.field     = V4L2_FIELD_INTERLACED;
                }
                if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                        errno_exit("VIDIOC_S_FMT");
                /* Note VIDIOC_S_FMT may change width and height. */
        } else {
                /* Preserve original settings as set by v4l2-ctl for example */
                if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                        errno_exit("VIDIOC_G_FMT");
        }
        /* Buggy driver paranoia. */
        min = fmt.fmt.pix.width * 2;
        if (fmt.fmt.pix.bytesperline < min)
                fmt.fmt.pix.bytesperline = min;
        min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
        if (fmt.fmt.pix.sizeimage < min)
                fmt.fmt.pix.sizeimage = min;
        switch (io) {
        case IO_METHOD_READ:
                init_read(fmt.fmt.pix.sizeimage);
                break;
        case IO_METHOD_MMAP:
                init_mmap();
                break;
        case IO_METHOD_USERPTR:
                init_userp(fmt.fmt.pix.sizeimage);
                break;
        }
}

static void close_device(void) {
        if (-1 == close(fd))
                errno_exit("close");
        fd = -1;
}

static void open_device(void) {
        struct stat st;
        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                        dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }
        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                        dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv) {
        fprintf(fp,
                "Usage: %s [options]\n\n"
                "Version 1.3\n"
                "Options:\n"
                "-d | --device name   Video device name [%s]\n"
                "-h | --help          Print this message\n"
                "-m | --mmap          Use memory mapped buffers [default]\n"
                "-r | --read          Use read() calls\n"
                "-u | --userp         Use application allocated buffers\n"
                "-o | --output        Outputs stream to stdout\n"
                 "-Y | --formatYUYV   Force format to 1280x720 YUYV\n"
                 "-H | --formatH264   Force format to 1280x720 H264\n"
                 "-R | --formatRGB3   Force format to 1280x720 RGB3\n"
                 "-c | --count        Number of frames to grab [%i] - use 0 for infinite\n"
                 "\n"
                 "Example usage: capture -H -o -c 300 > output.raw\n"
                 "Captures 300 frames of H264 at 1280x720 - use raw2mpg4 script to convert to mpg4\n",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruoYHRc:";

static const struct option
        long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "formatYUYV", no_argument,   NULL, 'Y' },
        { "formatH264", no_argument,   NULL, 'H' },
        { "formatRGB3", no_argument,   NULL, 'R' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

int main(int argc, char **argv) {
        overlayImage = read_png_file(overlayFilename.c_str());
        //readPNG(overlayFilename, overlayData.data(), startingWidth, startingHeight);
        dev_name = (char*)calloc(64, sizeof(char));
        strcpy(dev_name, "/dev/video0");
        for (;;) {
                int idx;
                int c;
                c = getopt_long(argc, argv,
                                short_options, long_options, &idx);
                if (-1 == c)
                        break;
                switch (c) {
                case 0: /* getopt_long() flag */
                        break;
                case 'd':
                        dev_name = optarg;
                        break;
                case 'h':
                        usage(stdout, argc, argv);
                        exit(EXIT_SUCCESS);
                case 'm':
                        io = IO_METHOD_MMAP;
                        break;
                case 'r':
                        io = IO_METHOD_READ;
                        break;
                case 'u':
                        io = IO_METHOD_USERPTR;
                        break;
                case 'o':
                        out_buf++;
                        break;
                case 'Y':
                        force_format=1;
                        break;
                case 'H':
                        force_format=2;
                        break;
                case 'R':
                        force_format=3;
                        break;
                case 'c':
                        errno = 0;
                        frame_count = strtol(optarg, NULL, 0);
                        if (errno)
                                errno_exit(optarg);
                        break;
                default:
                        usage(stderr, argc, argv);
                        exit(EXIT_FAILURE);
                }
        }
        open_device();
        init_device();
        start_capturing();
        mainloop();
        stop_capturing();
        uninit_device();
        close_device();
        fprintf(stderr, "\n");
        return 0;
}
