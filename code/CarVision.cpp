#include "CarVision.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static const int MODEL_SIZE = 96;

static const float CONFIDENCE_THRESH = 0.50f;
static const int INFER_INTERVAL = 2;

// ---- 红框检测参数（与 cai3.py 一致）----
// HSV 红色阈值
static const int H1_LOW = 0, S1_LOW = 168, V1_LOW = 123;
static const int H1_HIGH = 10, S1_HIGH = 255, V1_HIGH = 255;
static const int H2_LOW = 161, S2_LOW = 62, V2_LOW = 66;
static const int H2_HIGH = 180, S2_HIGH = 255, V2_HIGH = 255;

// 形态学核大小
static const int MORPH_KERNEL_SIZE = 2;

// 最小红色面积（像素）
static const int MIN_RED_AREA = 50;

// 长宽比限制
static const double MIN_ASPECT_RATIO = 0.3;
static const double MAX_ASPECT_RATIO = 5.0;

// 目标红色面积占画面比例
static const double TARGET_RED_AREA_RATIO = 0.002;

// 粘连判定阈值
static const int STICKY_AREA_MULT = 5;
static const int STICKY_MIN_HEIGHT = 20;
static const int EROSION_KERNEL_SIZE = 5;

// ===================== 内部结构 =====================
struct RedBox
{
    int x, y, w, h;
};

struct Candidate
{
    double area;
    double area_ratio;
    int x, y, w, h;
};

class CarVisionImpl
{
  public:
    CarVisionImpl();
    ~CarVisionImpl();

    bool init();
    bool updateFromFrame(const cv::Mat &img, int &category);
    int updateFromRgb565(const uint16_t *rgb565, int width, int height);
    void close();

  private:
    bool loadLabels();

    RedBox findTargetRedBox(const cv::Mat &frame);
    void trySplitStickyContour(const cv::Mat &mask, const cv::Rect &bbox, std::vector<Candidate> &candidates);
    bool computeRoiRect(const cv::Mat &frame, const RedBox &red_box, cv::Rect &roi_rect);
    cv::Mat cropRoi(const cv::Mat &frame, const RedBox &red_box);

    int inferRoi(const cv::Mat &roi, float &confidence);
    int mapToBigId(const std::string &label) const;

  private:
    ncnn::Net net_;
    std::vector<std::string> labels_;

    int frame_id_;
    int last_small_id_;
    int last_big_id_;
    float last_confidence_;

    uint8_t rgb565_to_r_[32];
    uint8_t rgb565_to_g_[64];
    uint8_t rgb565_to_b_[32];
};

// ===================== 实现 =====================
CarVisionImpl::CarVisionImpl() : frame_id_(0), last_small_id_(-1), last_big_id_(-1), last_confidence_(0.0f)
{
}

CarVisionImpl::~CarVisionImpl()
{
    close();
}

bool CarVisionImpl::init()
{
    for (int i = 0; i < 32; i++)
    {
        rgb565_to_r_[i] = (uint8_t)(i * 255 / 31);
        rgb565_to_b_[i] = (uint8_t)(i * 255 / 31);
    }
    for (int i = 0; i < 64; i++)
    {
        rgb565_to_g_[i] = (uint8_t)(i * 255 / 63);
    }

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

bool CarVisionImpl::updateFromFrame(const cv::Mat &img, int &category)
{
    category = -1;

    if (img.empty())
    {
        return false;
    }

    RedBox red_box = findTargetRedBox(img);
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

// ===================== 粘连拆分 =====================
void CarVisionImpl::trySplitStickyContour(const cv::Mat &mask, const cv::Rect &bbox, std::vector<Candidate> &candidates)
{
    if (bbox.width == 0 || bbox.height == 0 || bbox.x < 0 || bbox.y < 0 || bbox.x + bbox.width > mask.cols ||
        bbox.y + bbox.height > mask.rows)
    {
        return;
    }

    cv::Mat roi_mask = mask(bbox).clone();

    cv::Mat erosion_kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(EROSION_KERNEL_SIZE, EROSION_KERNEL_SIZE));
    cv::Mat roi_eroded;
    cv::erode(roi_mask, roi_eroded, erosion_kernel);

    cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
    cv::Mat roi_clean;
    cv::morphologyEx(roi_eroded, roi_clean, cv::MORPH_OPEN, open_kernel);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    int num_labels = cv::connectedComponentsWithStats(roi_clean, labels, stats, centroids, 8, CV_32S);

    std::vector<Candidate> sub_regions;

    for (int label_id = 1; label_id < num_labels; label_id++)
    {
        int sw = stats.at<int>(label_id, cv::CC_STAT_WIDTH);
        int sh = stats.at<int>(label_id, cv::CC_STAT_HEIGHT);
        int sx = stats.at<int>(label_id, cv::CC_STAT_LEFT);
        int sy = stats.at<int>(label_id, cv::CC_STAT_TOP);
        int eroded_area = stats.at<int>(label_id, cv::CC_STAT_AREA);

        if (eroded_area < MIN_RED_AREA || sw == 0 || sh == 0)
        {
            continue;
        }

        cv::Mat sub_mask = (labels == label_id);
        sub_mask.convertTo(sub_mask, CV_8U);
        sub_mask = sub_mask.mul(roi_mask);
        int actual_area = cv::countNonZero(sub_mask);

        int abs_x = bbox.x + sx;
        int abs_y = bbox.y + sy;

        double aspect_ratio = (sh > 0) ? (double)sw / sh : 0;
        if (aspect_ratio < MIN_ASPECT_RATIO || aspect_ratio > MAX_ASPECT_RATIO)
        {
            continue;
        }

        double area_ratio = (double)actual_area / (bbox.width * bbox.height);
        Candidate c;
        c.area = actual_area;
        c.area_ratio = area_ratio;
        c.x = abs_x;
        c.y = abs_y;
        c.w = sw;
        c.h = sh;
        sub_regions.push_back(c);
    }

    if (sub_regions.size() <= 1)
    {
        int actual_area = cv::countNonZero(roi_mask);
        double area_ratio = (double)actual_area / (bbox.width * bbox.height);
        double aspect_ratio = (bbox.height > 0) ? (double)bbox.width / bbox.height : 0;

        Candidate c;
        c.area = actual_area;
        c.area_ratio = area_ratio;
        c.x = bbox.x;
        c.y = bbox.y;
        c.w = bbox.width;
        c.h = bbox.height;
        candidates.push_back(c);
        return;
    }

    for (size_t i = 0; i < sub_regions.size(); i++)
    {
        candidates.push_back(sub_regions[i]);
    }
}

// ===================== 红框检测（与 cai3.py 逻辑一致）=====================
RedBox CarVisionImpl::findTargetRedBox(const cv::Mat &frame)
{
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(H1_LOW, S1_LOW, V1_LOW), cv::Scalar(H1_HIGH, S1_HIGH, V1_HIGH), mask1);
    cv::inRange(hsv, cv::Scalar(H2_LOW, S2_LOW, V2_LOW), cv::Scalar(H2_HIGH, S2_HIGH, V2_HIGH), mask2);

    mask = mask1 | mask2;

    int red_pixels = cv::countNonZero(mask);
    if (red_pixels == 0)
    {
        RedBox r = {0, 0, 0, 0};
        return r;
    }

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        RedBox r = {0, 0, 0, 0};
        return r;
    }

    double frame_area = frame.rows * frame.cols;
    std::vector<Candidate> candidates;

    for (size_t i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        cv::Rect bbox = cv::boundingRect(contours[i]);

        if (area >= MIN_RED_AREA * STICKY_AREA_MULT && bbox.height > STICKY_MIN_HEIGHT)
        {
            trySplitStickyContour(mask, bbox, candidates);
            continue;
        }

        if (area < MIN_RED_AREA)
        {
            continue;
        }

        if (bbox.width == 0 || bbox.height == 0)
        {
            continue;
        }

        double aspect_ratio = (double)bbox.width / bbox.height;
        if (aspect_ratio < MIN_ASPECT_RATIO || aspect_ratio > MAX_ASPECT_RATIO)
        {
            continue;
        }

        double area_ratio = area / frame_area;

        Candidate c;
        c.area = area;
        c.area_ratio = area_ratio;
        c.x = bbox.x;
        c.y = bbox.y;
        c.w = bbox.width;
        c.h = bbox.height;
        candidates.push_back(c);
    }

    if (candidates.empty())
    {
        RedBox r = {0, 0, 0, 0};
        return r;
    }

    double max_allowed_ratio = TARGET_RED_AREA_RATIO * 10.0;

    Candidate best = candidates[0];

    bool found_valid = false;
    for (size_t i = 0; i < candidates.size(); i++)
    {
        if (candidates[i].area_ratio <= max_allowed_ratio)
        {
            found_valid = true;
            break;
        }
    }

    if (found_valid)
    {
        double best_score = -1e30;
        for (size_t i = 0; i < candidates.size(); i++)
        {
            if (candidates[i].area_ratio > max_allowed_ratio)
            {
                continue;
            }
            double ar = (candidates[i].h > 0) ? (double)candidates[i].w / candidates[i].h : 0;
            double score = ar - std::abs(candidates[i].area_ratio - TARGET_RED_AREA_RATIO) * 10.0;
            if (score > best_score)
            {
                best_score = score;
                best = candidates[i];
            }
        }
    }
    else
    {
        double min_diff = 1e30;
        for (size_t i = 0; i < candidates.size(); i++)
        {
            double diff = std::abs(candidates[i].area_ratio - TARGET_RED_AREA_RATIO);
            if (diff < min_diff)
            {
                min_diff = diff;
                best = candidates[i];
            }
        }
    }

    RedBox result;
    result.x = best.x;
    result.y = best.y;
    result.w = best.w;
    result.h = best.h;
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

int CarVisionImpl::updateFromRgb565(const uint16_t *rgb565, int width, int height)
{
    if (rgb565 == nullptr || width <= 0 || height <= 0)
    {
        return -1;
    }

    cv::Mat frame(height, width, CV_8UC3);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            uint16_t pixel = rgb565[y * width + x];
            uint8_t r = rgb565_to_r_[(pixel >> 11) & 0x1F];
            uint8_t g = rgb565_to_g_[(pixel >> 5) & 0x3F];
            uint8_t b = rgb565_to_b_[pixel & 0x1F];
            frame.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        }
    }

    int category = -1;
    if (updateFromFrame(frame, category))
    {
        return category;
    }

    return -1;
}

void CarVisionImpl::close()
{
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

int vision_get_from_rgb565(const uint16_t *rgb565, int width, int height)
{
    if (g_vision == nullptr)
    {
        return -1;
    }

    return g_vision->updateFromRgb565(rgb565, width, height);
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