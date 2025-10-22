#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <bitset>
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;

// 每模块像素大小
const int MODULE_SIZE = 10;
const int BORDER = 4;

// 定位标记大小
const int FINDER_PATTERN_SIZE = 7;
const int FINDER_BORDER = 1;

// 读取二进制文件
vector<uint8_t> readBinaryFile(const string& filename) {
    ifstream ifs(filename, ios::binary);
    vector<uint8_t> data;
    if (!ifs) return data;
    ifs.seekg(0, ios::end);
    size_t size = ifs.tellg();
    ifs.seekg(0, ios::beg);
    data.resize(size);
    ifs.read(reinterpret_cast<char*>(data.data()), size);
    return data;
}

// 计算CRC32校验码
uint32_t calculateCRC32(const vector<uint8_t>& data) {
    uint32_t crc = 0xFFFFFFFF;
    uint32_t polynomial = 0xEDB88320;

    for (auto byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ polynomial;
            }
            else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

// 添加校验码到数据
vector<uint8_t> addChecksum(const vector<uint8_t>& data) {
    vector<uint8_t> result = data;
    uint32_t checksum = calculateCRC32(data);

    // 将32位校验码分成4个字节添加到数据末尾
    result.push_back((checksum >> 24) & 0xFF);
    result.push_back((checksum >> 16) & 0xFF);
    result.push_back((checksum >> 8) & 0xFF);
    result.push_back(checksum & 0xFF);

    return result;
}

// 添加奇偶校验，返回每字节 9bit（存储为 bool 向量）
vector<bool> addParityBits(const vector<uint8_t>& data) {
    vector<bool> bits;
    for (auto byte : data) {
        bitset<8> b(byte);
        bool parity = b.count() % 2; // 奇偶校验位，1 = 奇数个 1
        for (int i = 7; i >= 0; --i) {
            bits.push_back(b[i]);
        }
        bits.push_back(parity); // 加到末尾
    }
    return bits;
}

// 绘制定位标记
void drawFinderPattern(Mat& qrImage, int centerX, int centerY) {
    int startX = centerX - FINDER_PATTERN_SIZE / 2;
    int startY = centerY - FINDER_PATTERN_SIZE / 2;

    // 绘制外黑框
    for (int y = 0; y < FINDER_PATTERN_SIZE; ++y) {
        for (int x = 0; x < FINDER_PATTERN_SIZE; ++x) {
            int value = 0; // 黑色
            if (x >= FINDER_BORDER && x < FINDER_PATTERN_SIZE - FINDER_BORDER &&
                y >= FINDER_BORDER && y < FINDER_PATTERN_SIZE - FINDER_BORDER) {
                value = 255; // 内部白色
            }
            if (x >= FINDER_BORDER + 1 && x < FINDER_PATTERN_SIZE - FINDER_BORDER - 1 &&
                y >= FINDER_BORDER + 1 && y < FINDER_PATTERN_SIZE - FINDER_BORDER - 1) {
                value = 0; // 中心黑色
            }

            for (int py = 0; py < MODULE_SIZE; ++py) {
                for (int px = 0; px < MODULE_SIZE; ++px) {
                    int pixelX = (startX + x) * MODULE_SIZE + px;
                    int pixelY = (startY + y) * MODULE_SIZE + py;
                    if (pixelX >= 0 && pixelX < qrImage.cols &&
                        pixelY >= 0 && pixelY < qrImage.rows) {
                        qrImage.at<uint8_t>(pixelY, pixelX) = value;
                    }
                }
            }
        }
    }
}

// 绘制对齐标记（简化版）
void drawAlignmentPattern(Mat& qrImage, int centerX, int centerY) {
    int size = 5;
    int startX = centerX - size / 2;
    int startY = centerY - size / 2;

    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            int value = 0; // 黑色
            if (x >= 1 && x < size - 1 && y >= 1 && y < size - 1) {
                value = 255; // 白色
            }
            if (x >= 2 && x < size - 2 && y >= 2 && y < size - 2) {
                value = 0; // 中心黑色
            }

            for (int py = 0; py < MODULE_SIZE; ++py) {
                for (int px = 0; px < MODULE_SIZE; ++px) {
                    int pixelX = (startX + x) * MODULE_SIZE + px;
                    int pixelY = (startY + y) * MODULE_SIZE + py;
                    if (pixelX >= 0 && pixelX < qrImage.cols &&
                        pixelY >= 0 && pixelY < qrImage.rows) {
                        qrImage.at<uint8_t>(pixelY, pixelX) = value;
                    }
                }
            }
        }
    }
}

// 生成二维码图片
void encodeToQRCode(const vector<uint8_t>& data, const string& outImage) {
    // 添加校验码
    vector<uint8_t> dataWithChecksum = addChecksum(data);

    vector<bool> bits = addParityBits(dataWithChecksum);

    // 计算二维码模块数（最小正方形）
    //int moduleCount = ceil(sqrt(bits.size()));
    const float desired_aspect_ratio = 16.0 / 9.0;

    // 高度模块数量（向上取整，避免不足）
    int heightCount = ceil(sqrt(bits.size() / desired_aspect_ratio));

    // 宽度模块数量
    int widthCount = ceil(heightCount * desired_aspect_ratio);

    // 确保有足够空间放置定位标记
    //moduleCount = max(moduleCount, FINDER_PATTERN_SIZE + 2);
    heightCount = max(heightCount, FINDER_PATTERN_SIZE + 2);//height的更小

    // 计算实际二维码大小（包含边框）
    //int qrSize = moduleCount + 2 * BORDER;
    int qrWidthInModules = widthCount + 2 * BORDER;
    int qrHeightInModules = heightCount + 2 * BORDER;

    // 创建二维码图像
    //Mat qrImage = Mat::zeros(qrSize * MODULE_SIZE, qrSize * MODULE_SIZE, CV_8UC1);
    Mat qrImage = Mat(qrHeightInModules * MODULE_SIZE,
        qrWidthInModules * MODULE_SIZE, CV_8UC1);
    qrImage.setTo(255); // 白色背景
    
    // 绘制三个定位标记（左上、右上、左下）
    /*drawFinderPattern(qrImage, BORDER + FINDER_PATTERN_SIZE / 2,
        BORDER + FINDER_PATTERN_SIZE / 2);
    drawFinderPattern(qrImage, BORDER + moduleCount - FINDER_PATTERN_SIZE / 2 - 1,
        BORDER + FINDER_PATTERN_SIZE / 2);
    drawFinderPattern(qrImage, BORDER + FINDER_PATTERN_SIZE / 2,
        BORDER + moduleCount - FINDER_PATTERN_SIZE / 2 - 1);*/
    drawFinderPattern(qrImage, BORDER + FINDER_PATTERN_SIZE / 2,
        BORDER + FINDER_PATTERN_SIZE / 2);
    drawFinderPattern(qrImage,
        qrWidthInModules - BORDER - FINDER_PATTERN_SIZE / 2 - 1,
        BORDER + FINDER_PATTERN_SIZE / 2);
    drawFinderPattern(qrImage,
        BORDER + FINDER_PATTERN_SIZE / 2,
        qrHeightInModules - BORDER - FINDER_PATTERN_SIZE / 2 - 1);

    // 绘制对齐标记（右下角）
    /*if (moduleCount > 15) {
        drawAlignmentPattern(qrImage, BORDER + moduleCount - 4, BORDER + moduleCount - 4);
    }*/
    if (widthCount > 15 && heightCount > 15) {
        drawAlignmentPattern(
            qrImage,
            BORDER + widthCount - 4,
            BORDER + heightCount - 4
        );
    }

    // 绘制数据模块
    int idx = 0;
    for (int y = 0; y < heightCount; ++y) {
        for (int x = 0; x < widthCount; ++x) {
            // 跳过定位标记区域
            bool isFinderArea = false;

            // 左上定位标记
            if (x < FINDER_PATTERN_SIZE + FINDER_BORDER &&
                y < FINDER_PATTERN_SIZE + FINDER_BORDER) {
                isFinderArea = true;
            }
            // 右上定位标记
            if (x >= widthCount - FINDER_PATTERN_SIZE - FINDER_BORDER &&
                y < FINDER_PATTERN_SIZE + FINDER_BORDER) {
                isFinderArea = true;
            }
            // 左下定位标记
            if (x < FINDER_PATTERN_SIZE + FINDER_BORDER &&
                y >= heightCount - FINDER_PATTERN_SIZE - FINDER_BORDER) {
                isFinderArea = true;
            }
            // 对齐标记区域
            if (widthCount > 15 && heightCount > 15 &&
                x >= widthCount - 6 && x < widthCount - 2 &&
                y >= heightCount - 6 && y < heightCount - 2) {
                isFinderArea = true;
            }

            if (!isFinderArea && idx < bits.size()) {
                int value = bits[idx++] ? 0 : 255; // 黑 = 1, 白 = 0
                rectangle(qrImage,
                    Point((x + BORDER) * MODULE_SIZE, (y + BORDER) * MODULE_SIZE),
                    Point((x + BORDER + 1) * MODULE_SIZE - 1, (y + BORDER + 1) * MODULE_SIZE - 1),
                    Scalar(value), FILLED);
            }
        }
    }

    imwrite(outImage, qrImage);
    cout << "QRCode image generated: " << outImage << endl;
    cout << "Data size: " << data.size() << " bytes" << endl;
    cout << "With checksum: " << dataWithChecksum.size() << " bytes" << endl;
    cout << "QR Code size: " << widthCount << "x" << heightCount << " y" << endl;

    cout << "Total bits (with parity): " << bits.size() << endl;
    cout << "Bits actually written: " << idx << endl;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        cout << "Usage: encode <input_bin> <output_png>\n";
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = argv[2];

    vector<uint8_t> data = readBinaryFile(inputFile);
    if (data.empty()) {
        cout << "Error: Could not read input file or file is empty\n";
        return 1;
    }

    encodeToQRCode(data, outputFile);
    return 0;
}