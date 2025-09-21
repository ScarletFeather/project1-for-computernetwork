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

// �������һ�µĲ���
namespace CodeParams
{
    constexpr int FrameSize = 108;
    constexpr int BytesPerFrame = 1242;
    constexpr int SafeAreaWidth = 2;
    constexpr int QrPointSize = 18;
    constexpr int SmallQrPointbias = 6;
    constexpr int RectAreaCount = 7;

    // �����壨���������ȫһ�£�
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

// ��ǿ�Ķ�ά�붨λ����
vector<Point2f> findQRMarkers(const Mat& frame)
{
    vector<Point2f> markers;
    Mat gray, blurred, binary;

    // ת��Ϊ�Ҷ�ͼ
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    // ��˹ģ����������
    GaussianBlur(gray, blurred, Size(3, 3), 0);

    // ����Ӧ��ֵ����
    adaptiveThreshold(blurred, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY, 15, 5);

    // ��������
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

    // Ѱ������Ƕ�׵���������ά�붨λ���������
    vector<pair<Point2f, double>> markerCandidates;

    for (size_t i = 0; i < contours.size(); i++) {
        if (hierarchy[i][2] != -1) {  // ��������
            int childIdx = hierarchy[i][2];
            if (hierarchy[childIdx][2] != -1) {  // ����������������
                double area = contourArea(contours[i]);
                double perimeter = arcLength(contours[i], true);

                if (perimeter > 0) {
                    double circularity = (4 * CV_PI * area) / (perimeter * perimeter);

                    // ��λ���Ӧ���ǽ��������ε�
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

    // ���������
    sort(markerCandidates.begin(), markerCandidates.end(),
        [](const pair<Point2f, double>& a, const pair<Point2f, double>& b) {
            return a.second > b.second;
        });

    // ��Ӵ�ʶ��㣨�������ǰ������
    for (int i = 0; i < min(3, (int)markerCandidates.size()); i++) {
        markers.push_back(markerCandidates[i].first);
    }

    // ������½�Сʶ��㣨�����С�ĵ㣩
    if (markerCandidates.size() >= 4) {
        markers.push_back(markerCandidates.back().first);
    }
    else if (markerCandidates.size() == 3) {
        // ���û���ҵ�С�㣬ʹ�ô��������½�λ��
        Point2f avgCenter(0, 0);
        for (const auto& marker : markers) {
            avgCenter += marker;
        }
        avgCenter.x /= static_cast<float>(markers.size());
        avgCenter.y /= static_cast<float>(markers.size());

        // �ҵ�������½ǵĵ���ΪС��
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

// ��ǿ��͸��У������
Mat correctPerspective(const Mat& frame, const vector<Point2f>& markers)
{
    if (markers.size() < 4) {
        return Mat();
    }

    // �Ա�ǵ�����������ϣ����ϣ����£�����
    vector<Point2f> sortedMarkers = markers;

    // ��x��������
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

    // ��y�����������ҵ�
    sort(leftPoints.begin(), leftPoints.end(), [](const Point2f& a, const Point2f& b) {
        return a.y < b.y;
        });

    sort(rightPoints.begin(), rightPoints.end(), [](const Point2f& a, const Point2f& b) {
        return a.y < b.y;
        });

    // ����Դ�㼯�����ϣ����ϣ����£�����
    vector<Point2f> srcPoints;
    if (!leftPoints.empty() && !rightPoints.empty()) {
        srcPoints.push_back(leftPoints[0]); // ����
        srcPoints.push_back(rightPoints[0]); // ����
        srcPoints.push_back(leftPoints.back()); // ����
        srcPoints.push_back(rightPoints.back()); // ����
    }
    else {
        srcPoints = markers;
    }

    // Ŀ��㣺���ϣ����ϣ����£�����
    vector<Point2f> dstPoints = {
        Point2f(0, 0),
        Point2f(CodeParams::FrameSize, 0),
        Point2f(0, CodeParams::FrameSize),
        Point2f(CodeParams::FrameSize, CodeParams::FrameSize)
    };

    // ���㵥Ӧ�Ծ���
    Mat transform = findHomography(srcPoints, dstPoints, RANSAC, 10.0);

    if (transform.empty()) {
        return Mat();
    }

    // Ӧ��͸�ӱ任
    Mat corrected;
    warpPerspective(frame, corrected, transform,
        Size(CodeParams::FrameSize, CodeParams::FrameSize));

    return corrected;
}

// ��������������ȡ������ÿ֡1242�ֽڣ�
vector<uchar> extractDataFromQR(const Mat& correctedFrame)
{
    vector<uchar> frameData;
    Mat gray;
    cvtColor(correctedFrame, gray, COLOR_BGR2GRAY);

    // ʹ������Ӧ��ֵ����
    Mat binary;
    adaptiveThreshold(gray, binary, 255, ADAPTIVE_THRESH_GAUSSIAN_C,
        THRESH_BINARY, 15, 5);

    // ���ձ���������������ȡ����
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

                // ��ȡһ���ֽڣ�8�����أ�
                for (int bit = 0; bit < 8; bit++) {
                    int x = startX + colByte * 8 + bit;
                    int y = startY + row;

                    if (y < binary.rows && x < binary.cols) {
                        // ��ɫ���ش���1����ɫ����0
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

// �����뺯��
int main(int argc, char* argv[])
{
    if (argc != 4) {
        cerr << "Usage: decode <input_video> <output_bin> <validity_bin>" << endl;
        return 1;
    }

    string inputVideo = argv[1];
    string outputBin = argv[2];
    string validityBin = argv[3];

    // ����Ƶ�ļ�
    VideoCapture cap(inputVideo);
    if (!cap.isOpened()) {
        cerr << "Error: Cannot open video file: " << inputVideo << endl;
        return 1;
    }

    // ׼������ļ�
    ofstream outFile(outputBin, ios::binary);
    ofstream validFile(validityBin, ios::binary);

    if (!outFile.is_open() || !validFile.is_open()) {
        cerr << "Error: Cannot open output files" << endl;
        return 1;
    }

    // ������Ƶ��ÿһ֡
    Mat frame;
    int frameCount = 0;
    vector<uchar> allData;

    // ��ȡ��Ƶ֡��
    double fps = cap.get(CAP_PROP_FPS);
    if (fps <= 0) fps = 30;

    // ����Ԥ��������
    int totalFrames = cap.get(CAP_PROP_FRAME_COUNT);
    double duration = totalFrames / fps;
    size_t expectedData = static_cast<size_t>(totalFrames * CodeParams::BytesPerFrame);

    cout << "Video info: " << totalFrames << " frames, " << duration << " seconds" << endl;
    cout << "Expected data size: " << expectedData / (1024 * 1024) << " MB" << endl;

    while (cap.read(frame)) {
        frameCount++;

        // 1. ���Ҷ�ά�붨λ���
        vector<Point2f> markers = findQRMarkers(frame);

        if (markers.size() < 4) {
            cerr << "Warning: Found only " << markers.size() << " markers in frame " << frameCount << endl;
            continue;
        }

        // 2. ͸�ӱ任У��
        Mat corrected = correctPerspective(frame, markers);
        if (corrected.empty()) {
            cerr << "Warning: Perspective correction failed in frame " << frameCount << endl;
            continue;
        }

        // 3. ��У�����ͼ������ȡ���ݣ�ÿ֡1242�ֽڣ�
        vector<uchar> frameData = extractDataFromQR(corrected);

        if (frameData.size() != CodeParams::BytesPerFrame) {
            cerr << "Warning: Extracted data size mismatch in frame " << frameCount
                << ". Expected: " << CodeParams::BytesPerFrame
                << ", Got: " << frameData.size() << endl;
        }

        // 4. ����֡����
        allData.insert(allData.end(), frameData.begin(), frameData.end());

        // ���ȱ���
        if (frameCount % 10 == 0) {
            double progress = static_cast<double>(frameCount) / totalFrames * 100;
            size_t dataSizeMB = allData.size() / (1024 * 1024);
            cout << "Processed " << frameCount << " frames (" << fixed << setprecision(1)
                << progress << "%), Data: " << dataSizeMB << " MB" << endl;
        }
    }

    // д����������
    outFile.write(reinterpret_cast<const char*>(allData.data()), allData.size());

    // д����Ч����Ϣ���������ȫ�����Ϊ��Ч��
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

    // ������Դ
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