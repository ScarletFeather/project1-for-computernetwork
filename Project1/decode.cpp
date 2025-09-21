#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <limits>

using namespace cv;
using namespace std;

// 与编码器一致的参数
namespace CodeParams
{
    constexpr int FrameSize = 108;
    constexpr int BytesPerFrame = 1242;
    constexpr int SafeAreaWidth = 2;
    constexpr int QrPointSize = 18;
    constexpr int SmallQrPointbias = 6;
    constexpr int RectAreaCount = 7;

    // 区域定义（与编码器完全一致）
    const int lenlim[RectAreaCount] = { 138, 144, 648, 144, 144, 16, 8 };
    const int areapos[RectAreaCount][2][2] = {
        {{69, 16}, {QrPointSize + 3, SafeAreaWidth}},
        {{16, 72}, {SafeAreaWidth, QrPointSize}},
        {{72, 72}, {QrPointSize, QrPointSize}},
        {{72, 16}, {QrPointSize, FrameSize - QrPointSize}},
        {{16, 72}, {FrameSize - QrPointSize, QrPointSize}},
        {{8, 16}, {FrameSize - QrPointSize, FrameSize - QrPointSize}},
        {{8, 8}, {FrameSize - QrPointSize + 8, FrameSize - QrPointSize}}
    };
}

// 增强的二维码定位函数
vector<Point2f> findQRMarkers(const Mat& frame)
{
    vector<Point2f> markers;
    Mat gray, blurred, binary;

    // 转换为灰度图
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // 高斯模糊减少噪声
    GaussianBlur(gray, blurred, Size(3, 3), 0);

    // 自适应阈值处理
    adaptiveThreshold(blurred, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY, 15, 5);

    // 查找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // 寻找三层嵌套的轮廓（二维码定位标记特征）
    vector<pair<Point2f, double>> markerCandidates;

    for (size_t i = 0; i < contours.size(); i++) {
        if (hierarchy[i][2] != -1) {  // 有子轮廓
            int childIdx = hierarchy[i][2];
            if (hierarchy[childIdx][2] != -1) {  // 子轮廓还有子轮廓
                double area = contourArea(contours[i]);
                double perimeter = arcLength(contours[i], true);

                if (perimeter > 0) {
                    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);

                    // 定位标记应该是近似正方形的
                    if (circularity > 0.6 && area > 20) {
                        RotatedRect rect = minAreaRect(contours[i]);
                        markerCandidates.push_back({ rect.center, area });
                    }
                }
            }
        }
    }

    if (markerCandidates.empty()) {
        return markers;
    }

    // 按面积排序
    sort(markerCandidates.begin(), markerCandidates.end(),
        [](const pair<Point2f, double>& a, const pair<Point2f, double>& b) {
            return a.second > b.second;
        });

    // 添加大识别点（面积最大的前三个）
    for (int i = 0; i < min(3, (int)markerCandidates.size()); i++) {
        markers.push_back(markerCandidates[i].first);
    }

    // 添加右下角小识别点（面积最小的点）
    if (markerCandidates.size() >= 4) {
        markers.push_back(markerCandidates.back().first);
    }
    else if (markerCandidates.size() == 3) {
        // 如果没有找到小点，使用大点计算右下角位置
        Point2f avgCenter(0, 0);
        for (const auto& marker : markers) {
            avgCenter += marker;
        }
        avgCenter.x /= static_cast<float>(markers.size());
        avgCenter.y /= static_cast<float>(markers.size());

        // 找到最靠近右下角的点作为小点
        Point2f bottomRight(frame.cols, frame.rows);
        Point2f candidate;
        double minDist = numeric_limits<double>::max();

        for (const auto& marker : markerCandidates) {
            double dist = norm(marker.first - bottomRight);
            if (dist < minDist) {
                minDist = dist;
                candidate = marker.first;
            }
        }
        markers.push_back(candidate);
    }

    return markers;
}

// 增强的透视校正函数
Mat correctPerspective(const Mat& frame, const vector<Point2f>& markers)
{
    if (markers.size() < 4) {
        return Mat();
    }

    // 对标记点进行排序：左上，右上，左下，右下
    vector<Point2f> sortedMarkers = markers;

    // 按x坐标排序
    sort(sortedMarkers.begin(), sortedMarkers.end(), [](const Point2f& a, const Point2f& b) {
        return a.x < b.x;
        });

    vector<Point2f> leftPoints;
    vector<Point2f> rightPoints;

    for (const auto& pt : sortedMarkers) {
        if (pt.x < frame.cols / 2) {
            leftPoints.push_back(pt);
        }
        else {
            rightPoints.push_back(pt);
        }
    }

    // 按y坐标排序左右点
    sort(leftPoints.begin(), leftPoints.end(), [](const Point2f& a, const Point2f& b) {
        return a.y < b.y;
        });

    sort(rightPoints.begin(), rightPoints.end(), [](const Point2f& a, const Point2f& b) {
        return a.y < b.y;
        });

    // 构建源点集：左上，右上，左下，右下
    vector<Point2f> srcPoints;
    if (!leftPoints.empty() && !rightPoints.empty()) {
        srcPoints.push_back(leftPoints[0]); // 左上
        srcPoints.push_back(rightPoints[0]); // 右上
        srcPoints.push_back(leftPoints.back()); // 左下
        srcPoints.push_back(rightPoints.back()); // 右下
    }
    else {
        srcPoints = markers;
    }

    // 目标点：左上，右上，左下，右下
    vector<Point2f> dstPoints = {
        Point2f(0, 0),
        Point2f(CodeParams::FrameSize, 0),
        Point2f(0, CodeParams::FrameSize),
        Point2f(CodeParams::FrameSize, CodeParams::FrameSize)
    };

    // 计算单应性矩阵
    Mat transform = findHomography(srcPoints, dstPoints, RANSAC, 10.0);

    if (transform.empty()) {
        return Mat();
    }

    // 应用透视变换
    Mat corrected;
    warpPerspective(frame, corrected, transform,
        Size(CodeParams::FrameSize, CodeParams::FrameSize));

    return corrected;
}

// 高数据量比特提取函数（每帧1242字节）
vector<uchar> extractDataFromQR(const Mat& correctedFrame)
{
    vector<uchar> frameData;
    Mat gray;
    cvtColor(correctedFrame, gray, COLOR_BGR2GRAY);

    // 使用自适应阈值处理
    Mat binary;
    adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY, 15, 5);

    // 按照编码器的区域定义提取数据
    for (int areaID = 0; areaID < CodeParams::RectAreaCount; areaID++) {
        int rows = CodeParams::areapos[areaID][0][0];
        int cols = CodeParams::areapos[areaID][0][1];
        int startY = CodeParams::areapos[areaID][1][0];
        int startX = CodeParams::areapos[areaID][1][1];

        int bytesToExtract = CodeParams::lenlim[areaID];
        int extractedBytes = 0;

        for (int row = 0; row < rows && extractedBytes < bytesToExtract; row++) {
            for (int colByte = 0; colByte < cols / 8 && extractedBytes < bytesToExtract; colByte++) {
                uchar byte = 0;

                // 提取一个字节（8个比特）
                for (int bit = 0; bit < 8; bit++) {
                    int x = startX + colByte * 8 + bit;
                    int y = startY + row;

                    if (y < binary.rows && x < binary.cols) {
                        // 白色像素代表1，黑色代表0
                        bool pixelValue = (binary.at<uchar>(y, x) > 0);
                        byte |= (pixelValue << (7 - bit));
                    }
                }

                frameData.push_back(byte);
                extractedBytes++;
            }
        }
    }

    return frameData;
}

// 主解码函数
int main(int argc, char* argv[])
{
    if (argc != 4) {
        cerr << "Usage: decode <input_video> <output_bin> <validity_bin>" << endl;
        return 1;
    }

    string inputVideo = argv[1];
    string outputBin = argv[2];
    string validityBin = argv[3];

    // 打开视频文件
    VideoCapture cap(inputVideo);
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open video file: " << inputVideo << endl;
        return 1;
    }

    // 准备输出文件
    ofstream outFile(outputBin, ios::binary);
    ofstream validFile(validityBin, ios::binary);

    if (!outFile.is_open() || !validFile.is_open()) {
        cerr << "Error: Cannot open output files" << endl;
        return 1;
    }

    // 处理视频的每一帧
    Mat frame;
    int frameCount = 0;
    vector<uchar> allData;

    // 获取视频帧率
    double fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) fps = 30;

    // 计算预期数据量
    int totalFrames = cap.get(CAP_PROP_FRAME_COUNT);
    double duration = totalFrames / fps;
    size_t expectedData = static_cast<size_t>(totalFrames * CodeParams::BytesPerFrame);

    cout << "Video info: " << totalFrames << " frames, " << duration << " seconds" << endl;
    cout << "Expected data size: " << expectedData / (1024 * 1024) << " MB" << endl;

    while (cap.read(frame)) {
        frameCount++;

        // 1. 查找二维码定位标记
        vector<Point2f> markers = findQRMarkers(frame);

        if (markers.size() < 4) {
            cerr << "Warning: Found only " << markers.size() << " markers in frame " << frameCount << endl;
            continue;
        }

        // 2. 透视变换校正
        Mat corrected = correctPerspective(frame, markers);
        if (corrected.empty()) {
            cerr << "Warning: Perspective correction failed in frame " << frameCount << endl;
            continue;
        }

        // 3. 从校正后的图像中提取数据（每帧1242字节）
        vector<uchar> frameData = extractDataFromQR(corrected);

        if (frameData.size() != CodeParams::BytesPerFrame) {
            cerr << "Warning: Extracted data size mismatch in frame " << frameCount
                << ". Expected: " << CodeParams::BytesPerFrame
                << ", Got: " << frameData.size() << endl;
        }

        // 4. 保存帧数据
        allData.insert(allData.end(), frameData.begin(), frameData.end());

        // 进度报告
        if (frameCount % 10 == 0) {
            double progress = static_cast<double>(frameCount) / totalFrames * 100;
            size_t dataSizeMB = allData.size() / (1024 * 1024);
            cout << "Processed " << frameCount << " frames (" << fixed << setprecision(1)
                << progress << "%), Data: " << dataSizeMB << " MB" << endl;
        }
    }

    // 写入所有数据
    outFile.write(reinterpret_cast<const char*>(allData.data()), allData.size());

    // 写入有效性信息（简单起见，全部标记为有效）
    vector<bool> validity(allData.size(), true);
    for (size_t i = 0; i < validity.size(); i += 8) {
        char byte = 0;
        for (int j = 0; j < 8 && i + j < validity.size(); j++) {
            if (validity[i + j]) {
                byte |= (1 << (7 - j));
            }
        }
        validFile.write(&byte, 1);
    }

    // 清理资源
    cap.release();
    outFile.close();
    validFile.close();

    size_t totalBytes = allData.size();
    cout << "Decoding completed." << endl;
    cout << "Total frames processed: " << frameCount << endl;
    cout << "Total data extracted: " << totalBytes << " bytes ("
        << totalBytes / (1024.0 * 1024.0) << " MB)" << endl;
    cout << "Output written to: " << outputBin << " and " << validityBin << endl;

    return 0;
}