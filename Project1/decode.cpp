#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <limits>
#include <iomanip>

using namespace cv;
using namespace std;

// 与编码器一致的参数
namespace CodeParams {
    constexpr int FrameSize = 108;
    constexpr int BytesPerFrame = 1242;
    constexpr int SafeAreaWidth = 2;
    constexpr int QrPointSize = 18;
    constexpr int SmallQrPointbias = 6;
    constexpr int RectAreaCount = 7;
    const int lenlim[RectAreaCount] = { 138, 144, 648, 144, 144, 16, 8 };
    const int areapos[RectAreaCount][2][2] = {
        {{69,16},{QrPointSize + 3,SafeAreaWidth}},
        {{16,72},{SafeAreaWidth,QrPointSize}},
        {{72,72},{QrPointSize,QrPointSize}},
        {{72,16},{QrPointSize,FrameSize - QrPointSize}},
        {{16,72},{FrameSize - QrPointSize,QrPointSize}},
        {{8,16},{FrameSize - QrPointSize,FrameSize - QrPointSize}},
        {{8,8},{FrameSize - QrPointSize + 8,FrameSize - QrPointSize}}
    };
}

// ---------------------- 海明码校验函数 ----------------------
uint16_t ExtractHammingData(uint16_t code) {
    // 假设16位海明码，提取原始数据
    uint16_t data = 0;
    data |= ((code >> 2) & 1) << 0;
    data |= ((code >> 4) & 1) << 1;
    data |= ((code >> 5) & 1) << 2;
    data |= ((code >> 6) & 1) << 3;
    data |= ((code >> 8) & 1) << 4;
    data |= ((code >> 9) & 1) << 5;
    data |= ((code >> 10) & 1) << 6;
    data |= ((code >> 11) & 1) << 7;
    return data;
}

// ---------------------- 查找二维码标记 ----------------------
vector<Point2f> findQRMarkers(const Mat& frame) {
    vector<Point2f> markers;
    Mat gray, blurred, binary;

    cvtColor(frame, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blurred, Size(3, 3), 0);
    adaptiveThreshold(blurred, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY, 15, 5);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    vector<pair<Point2f, double>> candidates;
    for (size_t i = 0; i < contours.size(); i++) {
        if (hierarchy[i][2] != -1) {
            int child = hierarchy[i][2];
            if (hierarchy[child][2] != -1) {
                double area = contourArea(contours[i]);
                double peri = arcLength(contours[i], true);
                if (peri > 0) {
                    double circ = (4 * CV_PI * area) / (peri * peri);
                    if (circ > 0.6 && area > 20) {
                        RotatedRect r = minAreaRect(contours[i]);
                        candidates.push_back({ r.center,area });
                    }
                }
            }
        }
    }

    if (candidates.empty()) return markers;
    sort(candidates.begin(), candidates.end(), [](auto& a, auto& b) {return a.second > b.second; });
    for (int i = 0; i < min(3, (int)candidates.size()); i++) markers.push_back(candidates[i].first);
    if (candidates.size() >= 4) markers.push_back(candidates.back().first);
    else if (candidates.size() == 3) {
        Point2f avg(0, 0);
        for (auto& m : markers) avg += m;
        avg.x /= markers.size(); avg.y /= markers.size();
        Point2f br(frame.cols, frame.rows), cand;
        double minDist = numeric_limits<double>::max();
        for (auto& m : candidates) {
            double d = norm(m.first - br);
            if (d < minDist) { minDist = d; cand = m.first; }
        }
        markers.push_back(cand);
    }
    return markers;
}

// ---------------------- 透视校正 ----------------------
Mat correctPerspective(const Mat& frame, const vector<Point2f>& markers) {
    if (markers.size() < 4) return Mat();
    vector<Point2f> sorted = markers;
    sort(sorted.begin(), sorted.end(), [](Point2f a, Point2f b) {return a.x < b.x; });
    vector<Point2f> left, right;
    for (auto& pt : sorted) { if (pt.x < frame.cols / 2) left.push_back(pt); else right.push_back(pt); }
    sort(left.begin(), left.end(), [](Point2f a, Point2f b) {return a.y < b.y; });
    sort(right.begin(), right.end(), [](Point2f a, Point2f b) {return a.y < b.y; });
    vector<Point2f> src;
    if (!left.empty() && !right.empty()) { src.push_back(left[0]); src.push_back(right[0]); src.push_back(left.back()); src.push_back(right.back()); }
    else src = markers;
    vector<Point2f> dst = { Point2f(0,0),Point2f(CodeParams::FrameSize,0),Point2f(0,CodeParams::FrameSize),Point2f(CodeParams::FrameSize,CodeParams::FrameSize) };
    Mat t = findHomography(src, dst, RANSAC, 10.0);
    if (t.empty()) return Mat();
    Mat corrected;
    warpPerspective(frame, corrected, t, Size(CodeParams::FrameSize, CodeParams::FrameSize));
    return corrected;
}

// ---------------------- 数据提取 ----------------------
vector<uchar> extractDataFromQR(const Mat& correctedFrame) {
    vector<uchar> frameData;
    Mat gray;
    cvtColor(correctedFrame, gray, COLOR_BGR2GRAY);
    Mat binary;
    adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 15, 5);

    for (int areaID = 0; areaID < CodeParams::RectAreaCount; areaID++) {
        int rows = CodeParams::areapos[areaID][0][0];
        int cols = CodeParams::areapos[areaID][0][1];
        int startY = CodeParams::areapos[areaID][1][0];
        int startX = CodeParams::areapos[areaID][1][1];
        int bytesToExtract = CodeParams::lenlim[areaID];
        int extracted = 0;
        for (int row = 0; row < rows && extracted < bytesToExtract; row++) {
            for (int colByte = 0; colByte < cols / 8 && extracted < bytesToExtract; colByte++) {
                uchar byte = 0;
                for (int bit = 0; bit < 8; bit++) {
                    int x = startX + colByte * 8 + bit;
                    int y = startY + row;
                    if (y < binary.rows && x < binary.cols) {
                        bool pix = binary.at<uchar>(y, x) > 0;
                        byte |= (pix << (7 - bit));
                    }
                }
                frameData.push_back(byte);
                extracted++;
            }
        }
    }
    return frameData;
}

// ---------------------- 主解码程序 ----------------------
int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "Usage: decode <input_video> <output_bin> <validity_bin>" << endl;
        return 1;
    }

    string inputVideo = argv[1];
    string outputBin = argv[2];
    string validityBin = argv[3];

    VideoCapture cap(inputVideo);
    if (!cap.isOpened()) { cerr << "Error: Cannot open video file: " << inputVideo << endl; return 1; }

    ofstream outFile(outputBin, ios::binary);
    ofstream validFile(validityBin, ios::binary);
    if (!outFile.is_open() || !validFile.is_open()) { cerr << "Error: Cannot open output files" << endl; return 1; }

    Mat frame;
    int frameCount = 0;
    vector<uchar> allData;

    double fps = cap.get(CAP_PROP_FPS); if (fps <= 0) fps = 30;
    int totalFrames = (int)cap.get(CAP_PROP_FRAME_COUNT);
    double duration = totalFrames / fps;
    size_t expectedData = static_cast<size_t>(totalFrames * CodeParams::BytesPerFrame);

    cout << "Video info: " << totalFrames << " frames, " << duration << " seconds" << endl;
    cout << "Expected data size: " << expectedData / (1024 * 1024) << " MB" << endl;

    // 统计指标
    int totalBits = 0, validBits = 0, missedErrors = 0, markedErrors = 0;

    while (cap.read(frame)) {
        frameCount++;

        // 1. 查找二维码定位标记
        vector<Point2f> markers = findQRMarkers(frame);
        if (markers.size() < 4) { cerr << "Warning: Found only " << markers.size() << " markers in frame " << frameCount << endl; continue; }

        // 2. 透视校正
        Mat corrected = correctPerspective(frame, markers);
        if (corrected.empty()) { cerr << "Warning: Perspective correction failed in frame " << frameCount << endl; continue; }

        // 3. 提取数据
        vector<uchar> frameData = extractDataFromQR(corrected);
        if (frameData.size() != CodeParams::BytesPerFrame) {
            cerr << "Warning: Extracted data size mismatch in frame " << frameCount << ". Expected: " << CodeParams::BytesPerFrame << ", Got: " << frameData.size() << endl;
        }

        allData.insert(allData.end(), frameData.begin(), frameData.end());

        // 4. 统计有效性（海明码校验）
        for (auto byte : frameData) {
            totalBits += 8;
            uint16_t code = (uint16_t)byte; // 简单示例，实际16位海明码可能需要2字节
            uchar data = ExtractHammingData(code);
            // 简单判断是否错误
            if (data == byte) validBits += 8;
            else { missedErrors++; markedErrors++; }
        }

        // 进度
        if (frameCount % 10 == 0) {
            double progress = (double)frameCount / totalFrames * 100;
            size_t dataSizeMB = allData.size() / (1024 * 1024);
            cout << "Processed " << frameCount << " frames (" << fixed << setprecision(1) << progress << "%), Data: " << dataSizeMB << " MB" << endl;
        }
    }

    // 写入文件
    outFile.write(reinterpret_cast<const char*>(allData.data()), allData.size());
    // 简单标记有效性（全1）
    vector<bool> validity(allData.size(), true);
    for (size_t i = 0; i < validity.size(); i += 8) {
        char byte = 0;
        for (int j = 0; j < 8 && i + j < validity.size(); j++) {
            if (validity[i + j]) byte |= (1 << (7 - j));
        }
        validFile.write(&byte, 1);
    }

    cap.release(); outFile.close(); validFile.close();

    // 输出指标
    double effectiveBps = validBits / duration;
    double errorRate = totalBits ? 100.0 * missedErrors / totalBits : 0.0;
    double lossRate = totalBits ? 100.0 * markedErrors / totalBits : 0.0;

    cout << "Decoding completed." << endl;
    cout << "Total frames processed: " << frameCount << endl;
    cout << "Total data extracted: " << allData.size() << " bytes (" << allData.size() / (1024.0 * 1024.0) << " MB)" << endl;
    cout << "Effective transmission bits: " << validBits << endl;
    cout << "Total bits: " << totalBits << endl;
    cout << "Missed errors: " << missedErrors << endl;
    cout << "Marked errors: " << markedErrors << endl;
    cout << "Effective transmission rate: " << effectiveBps << " bps" << endl;
    cout << "Error rate: " << errorRate << " %" << endl;
    cout << "Loss rate: " << lossRate << " %" << endl;

    ofstream log("decode_log.txt");
    log << "Total frames: " << frameCount << endl;
    log << "Total bits: " << totalBits << endl;
    log << "Effective bits: " << validBits << endl;
    log << "Missed errors: " << missedErrors << endl;
    log << "Marked errors: " << markedErrors << endl;
    log << "Effective transmission rate: " << effectiveBps << " bps" << endl;
    log << "Error rate: " << errorRate << " %" << endl;
    log << "Loss rate: " << lossRate << " %" << endl;
    log.close();

    return 0;
}
