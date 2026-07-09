#include "CarVision.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ncnn/net.h>
#include <opencv2/opencv.hpp>

// ===================== 配置 =====================
static const char *MODEL_PARAM = "tiny_classifier_6fp32.ncnn.param";
static const char *MODEL_BIN = "tiny_classifier_6fp32.ncnn.bin";
static const char *LABELS_PATH = "labels6.txt";
static const char *CAMERA_DEV = "/dev/video0";

static const int FRAME_WIDTH = 320;
static const int FRAME_HEIGHT = 240;
static const int MODEL_SIZE = 96;

static const float CONFIDENCE_THRESH = 0.60f;
static const int INFER_INTERVAL = 2;
static const int MIN_RED_AREA = 25;
static const int SELECT_TIMEOUT_MS = 10;

// ===================== 内部类 =====================
struct RedBox
{
    int x, y, w, h;
};

class CarVisionImpl
{
  public:
    CarVisionImpl();
    ~CarVisionImpl();

    bool init();
    bool update(int &category);
    bool updateFromFrame(const cv::Mat &img, int &category);
    void close();

  private:
    bool loadLabels();
    bool initCamera();
    cv::Mat captureFrame();

    RedBox findLargestRedBox(const cv::Mat &frame);
    bool computeRoiRect(const cv::Mat &frame, const RedBox &red_box, cv::Rect &roi_rect);
    cv::Mat cropRoi(const cv::Mat &frame, const RedBox &red_box);

    int inferRoi(const cv::Mat &roi, float &confidence);
    int mapToBigId(const std::string &label) const;

  private:
    ncnn::Net net_;
    std::vector<std::string> labels_;

    int fd_;
    unsigned char *framebuf_;
    unsigned int framebuf_length_;
    bool camera_started_;

    int frame_id_;
    int last_small_id_;
    int last_big_id_;
    float last_confidence_;
};

// ===================== 实现 =====================
CarVisionImpl::CarVisionImpl()
    : fd_(-1), framebuf_(nullptr), framebuf_length_(0), camera_started_(false), frame_id_(0), last_small_id_(-1),
      last_big_id_(-1), last_confidence_(0.0f)
{
}

CarVisionImpl::~CarVisionImpl()
{
    close();
}

bool CarVisionImpl::init()
{
    net_.opt.num_threads = 2;
    net_.opt.use_packing_layout = true;
    net_.opt.lightmode = true;

    int ret_param = net_.load_param(MODEL_PARAM);
    if (ret_param != 0)
    {
        printf("加载 param 失败: %s, ret=%d\n", MODEL_PARAM, ret_param);
        return false;
    }

    int ret_bin = net_.load_model(MODEL_BIN);
    if (ret_bin != 0)
    {
        printf("加载 bin 失败: %s, ret=%d\n", MODEL_BIN, ret_bin);
        return false;
    }

    if (!loadLabels())
    {
        printf("加载 labels 失败: %s\n", LABELS_PATH);
        return false;
    }

    printf("视觉初始化成功，labels=%zu\n", labels_.size());
    return true;
}

bool CarVisionImpl::loadLabels()
{
    labels_.clear();

    FILE *fp = fopen(LABELS_PATH, "r");
    if (!fp)
    {
        return false;
    }

    char line[128];
    while (fgets(line, sizeof(line), fp))
    {
        line[strcspn(line, "\r\n")] = 0;
        if (strlen(line) > 0)
        {
            labels_.push_back(line);
        }
    }

    fclose(fp);
    return !labels_.empty();
}

bool CarVisionImpl::initCamera()
{
    fd_ = open(CAMERA_DEV, O_RDWR);
    if (fd_ < 0)
    {
        perror("open camera");
        return false;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = FRAME_WIDTH;
    fmt.fmt.pix.height = FRAME_HEIGHT;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
    {
        perror("VIDIOC_S_FMT");
        close();
        return false;
    }

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0)
    {
        perror("VIDIOC_REQBUFS");
        close();
        return false;
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
    {
        perror("VIDIOC_QUERYBUF");
        close();
        return false;
    }

    framebuf_length_ = buf.length;
    framebuf_ = (unsigned char *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

    if (framebuf_ == MAP_FAILED)
    {
        perror("mmap");
        framebuf_ = nullptr;
        close();
        return false;
    }

    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0)
    {
        perror("VIDIOC_QBUF");
        close();
        return false;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
    {
        perror("VIDIOC_STREAMON");
        close();
        return false;
    }

    camera_started_ = true;
    return true;
}

cv::Mat CarVisionImpl::captureFrame()
{
    if (fd_ < 0 || !camera_started_)
    {
        return cv::Mat();
    }

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    struct timeval tv;
    tv.tv_sec = SELECT_TIMEOUT_MS / 1000;
    tv.tv_usec = (SELECT_TIMEOUT_MS % 1000) * 1000;

    int ret = select(fd_ + 1, &fds, NULL, NULL, &tv);
    if (ret <= 0)
    {
        return cv::Mat();
    }

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
    {
        return cv::Mat();
    }

    cv::Mat img = cv::imdecode(cv::Mat(1, buf.bytesused, CV_8UC1, framebuf_), cv::IMREAD_COLOR);

    ioctl(fd_, VIDIOC_QBUF, &buf);
    return img;
}

bool CarVisionImpl::update(int &category)
{
    category = -1;

    cv::Mat img = captureFrame();
    if (img.empty())
    {
        return false;
    }

    return updateFromFrame(img, category);
}

bool CarVisionImpl::updateFromFrame(const cv::Mat &img, int &category)
{
    category = -1;

    if (img.empty())
    {
        return false;
    }

    RedBox red_box = findLargestRedBox(img);
    bool has_red_box = (red_box.w > 0 && red_box.h > 0);

    if (!has_red_box)
    {
        last_small_id_ = -1;
        last_big_id_ = -1;
        last_confidence_ = 0.0f;
        frame_id_++;
        return false;
    }

    cv::Mat roi = cropRoi(img, red_box);
    if (roi.empty())
    {
        frame_id_++;
        return false;
    }

    int small_id = -1;
    float confidence = 0.0f;

    if (frame_id_ % INFER_INTERVAL == 0 || last_small_id_ < 0)
    {
        small_id = inferRoi(roi, confidence);
        last_small_id_ = small_id;
        last_confidence_ = confidence;

        if (small_id >= 0 && small_id < (int)labels_.size())
        {
            last_big_id_ = mapToBigId(labels_[small_id]);
        }
        else
        {
            last_big_id_ = -1;
        }
    }
    else
    {
        small_id = last_small_id_;
        confidence = last_confidence_;
    }

    frame_id_++;

    if (small_id < 0 || small_id >= (int)labels_.size())
    {
        return false;
    }

    if (confidence < CONFIDENCE_THRESH)
    {
        return false;
    }

    if (last_big_id_ < 0)
    {
        return false;
    }

    category = last_big_id_;
    return true;
}

RedBox CarVisionImpl::findLargestRedBox(const cv::Mat &frame)
{
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 80, 90), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(180, 255, 255), mask2);

    mask = mask1 | mask2;

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    RedBox result = {0, 0, 0, 0};
    double max_area = 0.0;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area < MIN_RED_AREA)
        {
            continue;
        }

        if (area > max_area)
        {
            max_area = area;
            cv::Rect rect = cv::boundingRect(contours[i]);
            result.x = rect.x;
            result.y = rect.y;
            result.w = rect.width;
            result.h = rect.height;
        }
    }

    return result;
}

bool CarVisionImpl::computeRoiRect(const cv::Mat &frame, const RedBox &red_box, cv::Rect &roi_rect)
{
    if (frame.empty() || red_box.w <= 0 || red_box.h <= 0)
    {
        return false;
    }

    int square_size = std::max(red_box.w, red_box.h);
    if (square_size <= 0)
    {
        return false;
    }

    if (square_size > frame.cols || square_size > frame.rows)
    {
        return false;
    }

    int roi_x = red_box.x + (red_box.w - square_size) / 2;
    int roi_y = red_box.y - square_size;

    roi_x = std::max(0, std::min(roi_x, frame.cols - square_size));
    roi_y = std::max(0, std::min(roi_y, frame.rows - square_size));

    roi_rect = cv::Rect(roi_x, roi_y, square_size, square_size);
    return true;
}

cv::Mat CarVisionImpl::cropRoi(const cv::Mat &frame, const RedBox &red_box)
{
    cv::Rect roi_rect;
    if (!computeRoiRect(frame, red_box, roi_rect))
    {
        return cv::Mat();
    }

    return frame(roi_rect).clone();
}

int CarVisionImpl::inferRoi(const cv::Mat &roi, float &confidence)
{
    confidence = 0.0f;

    if (roi.empty())
    {
        return -1;
    }

    cv::Mat rgb, resized;
    cv::cvtColor(roi, rgb, cv::COLOR_BGR2RGB);
    cv::resize(rgb, resized, cv::Size(MODEL_SIZE, MODEL_SIZE));

    ncnn::Mat in = ncnn::Mat(MODEL_SIZE, MODEL_SIZE, 3);

    for (int y = 0; y < MODEL_SIZE; y++)
    {
        for (int x = 0; x < MODEL_SIZE; x++)
        {
            cv::Vec3b pixel = resized.at<cv::Vec3b>(y, x);

            float r = pixel[0] / 255.0f;
            float g = pixel[1] / 255.0f;
            float b = pixel[2] / 255.0f;

            r = (r - 0.485f) / 0.229f;
            g = (g - 0.456f) / 0.224f;
            b = (b - 0.406f) / 0.225f;

            in.channel(0)[y * MODEL_SIZE + x] = r;
            in.channel(1)[y * MODEL_SIZE + x] = g;
            in.channel(2)[y * MODEL_SIZE + x] = b;
        }
    }

    ncnn::Extractor ex = net_.create_extractor();
    ex.input("in0", in);

    ncnn::Mat out;
    ex.extract("out0", out);

    if (out.w <= 0)
    {
        return -1;
    }

    std::vector<float> probs(out.w);

    float max_val = out[0];
    for (int i = 1; i < out.w; i++)
    {
        if (out[i] > max_val)
        {
            max_val = out[i];
        }
    }

    float sum = 0.0f;
    for (int i = 0; i < out.w; i++)
    {
        probs[i] = std::exp(out[i] - max_val);
        sum += probs[i];
    }

    if (sum <= 0.0f)
    {
        return -1;
    }

    int class_id = 0;
    float max_prob = 0.0f;
    for (int i = 0; i < out.w; i++)
    {
        probs[i] /= sum;
        if (probs[i] > max_prob)
        {
            max_prob = probs[i];
            class_id = i;
        }
    }

    confidence = max_prob;
    return class_id;
}

int CarVisionImpl::mapToBigId(const std::string &label) const
{
    if (label == "qiang" || label == "dan" || label == "weapon")
    {
        return 0;
    }
    if (label == "ji" || label == "wang" || label == "supplies")
    {
        return 1;
    }
    if (label == "jiu" || label == "zhuang" || label == "vehicle")
    {
        return 2;
    }
    return -1;
}

void CarVisionImpl::close()
{
    if (fd_ >= 0 && camera_started_)
    {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        camera_started_ = false;
    }

    if (framebuf_ != nullptr)
    {
        munmap(framebuf_, framebuf_length_);
        framebuf_ = nullptr;
        framebuf_length_ = 0;
    }

    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

// ===================== 全局接口 =====================
static CarVisionImpl *g_vision = nullptr;

bool vision_init()
{
    if (g_vision != nullptr)
    {
        return true;
    }

    g_vision = new CarVisionImpl();
    if (!g_vision->init())
    {
        delete g_vision;
        g_vision = nullptr;
        return false;
    }

    return true;
}

int vision_get()
{
    if (g_vision == nullptr)
    {
        return -1;
    }

    int category = -1;

    if (g_vision->update(category))
    {
        return category;
    }

    return -1;
}

int vision_get_from_rgb565(const uint16_t *rgb565, int width, int height)
{
    if (g_vision == nullptr || rgb565 == nullptr || width <= 0 || height <= 0)
    {
        return -1;
    }

    cv::Mat frame(height, width, CV_8UC3);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uint16_t pixel = rgb565[y * width + x];
            uint8_t r = (uint8_t)(((pixel >> 11) & 0x1F) * 255 / 31);
            uint8_t g = (uint8_t)(((pixel >> 5) & 0x3F) * 255 / 63);
            uint8_t b = (uint8_t)((pixel & 0x1F) * 255 / 31);
            frame.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        }
    }

    int category = -1;
    if (g_vision->updateFromFrame(frame, category))
    {
        return category;
    }

    return -1;
}

void vision_close()
{
    if (g_vision != nullptr)
    {
        g_vision->close();
        delete g_vision;
        g_vision = nullptr;
    }
}
