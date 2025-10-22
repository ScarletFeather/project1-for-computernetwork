#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <bitset>

using namespace cv;
using namespace std;

// 与编码器完全相同的参数
const int MODULE_SIZE = 10;
const int BORDER = 4;
const int FINDER_PATTERN_SIZE = 7;
const int FINDER_BORDER = 1;

// 写二进制文件
void writeBinaryFile(const string& filename, const vector<uint8_t>& data) {
    ofstream ofs(filename, ios::binary);
    ofs.write(reinterpret_cast<const char*>(data.data()), data.size());
}

// 检测定位标记区域（与编码器完全一致）
bool isFinderPatternArea(int x, int y, int widthCount,int heightCount) {
    // 左上定位标记区域
    if (x < FINDER_PATTERN_SIZE + FINDER_BORDER &&
        y < FINDER_PATTERN_SIZE + FINDER_BORDER) {
        return true;
    }
    // 右上定位标记区域
    if (x >= widthCount - FINDER_PATTERN_SIZE - FINDER_BORDER &&
        y < FINDER_PATTERN_SIZE + FINDER_BORDER) {
        return true;
    }
    // 左下定位标记区域
    if (x < FINDER_PATTERN_SIZE + FINDER_BORDER &&
        y >= heightCount - FINDER_PATTERN_SIZE - FINDER_BORDER) {
        return true;
    }
    // 对齐标记区域（如果存在）
    if (widthCount > 15 && heightCount>15&&
        x >= widthCount - 6 && x < widthCount - 2 &&
        y >= heightCount - 6 && y < heightCount - 2) {
        return true;
    }

    return false;
}

// 验证CRC32校验码（与编码器完全一致）
bool verifyChecksum(vector<uint8_t>& data) {
    if (data.size() < 4) {
        cout << "Data too short for checksum verification" << endl;
        return false;
    }

    // 提取校验码
    uint32_t storedChecksum = (data[data.size() - 4] << 24) |
        (data[data.size() - 3] << 16) |
        (data[data.size() - 2] << 8) |
        data[data.size() - 1];

    // 移除校验码计算实际数据的CRC
    vector<uint8_t> actualData(data.begin(), data.end() - 4);

    // 计算CRC32（与编码器相同的算法）
    uint32_t crc = 0xFFFFFFFF;
    uint32_t polynomial = 0xEDB88320;

    for (auto byte : actualData) {
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
    crc = ~crc;

    bool valid = (crc == storedChecksum);
    if (valid) {
        cout << "Checksum verification passed" << endl;
        data = actualData; // 移除校验码，返回原始数据
    }
    else {
        cout << "Checksum verification failed" << endl;
        cout << "Expected: " << hex << storedChecksum << ", Got: " << crc << dec << endl;
    }

    return valid;
}

// 计算模块数量（与编码器逻辑一致）
//int calculateModuleCount(int imageSize) {
//    int totalModulesWithBorder = imageSize / MODULE_SIZE;
//    int moduleCount = totalModulesWithBorder - 2 * BORDER;
//    return moduleCount;
//}
int calculateWidthModuleCount(int imageWidth) {
    return (imageWidth / MODULE_SIZE) - 2 * BORDER;
}

int calculateHeightModuleCount(int imageHeight) {
    return (imageHeight / MODULE_SIZE) - 2 * BORDER;
}

// 改进的有效性检测：使用3x3区域投票来确定每个比特的可信度
uint8_t calculateBitValidity(const Mat& qrImage, int centerX, int centerY) {
    int vote = 0;
    int total = 0;

    // 检查3x3区域
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            int px = centerX + i * (MODULE_SIZE / 3);
            int py = centerY + j * (MODULE_SIZE / 3);

            if (px >= 0 && px < qrImage.cols && py >= 0 && py < qrImage.rows) {
                uchar val = qrImage.at<uchar>(py, px);
                vote += (val < 128) ? 1 : 0; // 黑色像素计数
                total++;
            }
        }
    }

    if (total == 0) return 0;

    // 计算一致性分数 (0-255)
    double consistency = 1.0 - (2.0 * abs(vote - total / 2.0) / total);
    uint8_t validity = static_cast<uint8_t>(consistency * 255);

    return validity;
}

// 解码二维码（与编码器完全匹配）
vector<uint8_t> decodeQRCode(const Mat& qrImage, vector<uint8_t>& validity) {
    // 计算模块数量（与编码器相同逻辑）
    //int imgSize = qrImage.rows;
    //int moduleCount = calculateModuleCount(imgSize);
    int imgWidth = qrImage.cols;
    int imgHeight = qrImage.rows;

    int widthModules = calculateWidthModuleCount(imgWidth);
    int heightModules = calculateHeightModuleCount(imgHeight);

    cout << "Module count: " << widthModules << " x " << heightModules << endl;

    cout << "Image size: " << imgWidth << "x" << imgHeight<<"y" << endl;
    cout << "Module size: " << MODULE_SIZE << endl;
    //cout << "Module count: " << moduleCount << endl;

    vector<bool> bits;
    validity.clear();

    // 按行扫描，跳过定位标记区域（与编码器写入顺序完全一致）
    for (int y = 0; y < heightModules; ++y) {
        for (int x = 0; x < widthModules; ++x) {
            // 跳过定位标记区域（与编码器完全一致）
            if (isFinderPatternArea(x, y, widthModules,heightModules)) {
                continue;
            }

            // 读取模块中心点的值
            int centerX = (x + BORDER) * MODULE_SIZE + MODULE_SIZE / 2;
            int centerY = (y + BORDER) * MODULE_SIZE + MODULE_SIZE / 2;

            if (centerX >= qrImage.cols) centerX = qrImage.cols - 1;
            if (centerY >= qrImage.rows) centerY = qrImage.rows - 1;

            uchar pixelValue = qrImage.at<uchar>(centerY, centerX);
            bool bit = (pixelValue < 128); // 黑色为1，白色为0

            bits.push_back(bit);

            // 改进：按比特计算有效性
            uint8_t bitValidity = calculateBitValidity(qrImage, centerX, centerY);
            validity.push_back(bitValidity);
        }
    }

    cout << "Total bits extracted: " << bits.size() << endl;
    cout << "Total validity bytes: " << validity.size() << endl;

    // 9位一组解码（8位数据 + 1位奇偶校验）
    vector<uint8_t> bytes;
    for (size_t i = 0; i + 8 < bits.size(); i += 9) {
        // 提取8位数据
        uint8_t byte = 0;
        for (int j = 0; j < 8; ++j) {
            byte = (byte << 1) | (bits[i + j] ? 1 : 0);
        }

        // 检查奇偶校验位
        bool parityBit = bits[i + 8];
        bool calculatedParity = (bitset<8>(byte).count() % 2 == 1);

        if (parityBit != calculatedParity) {
            cout << "Parity error at byte group starting at bit " << i << endl;
            cout << "Data: " << bitset<8>(byte) << ", Parity bit: " << parityBit
                << ", Expected parity: " << calculatedParity << endl;
        }

        bytes.push_back(byte);
    }

    cout << "Decoded bytes (with checksum): " << bytes.size() << endl;

    // 验证CRC32校验码
    if (!bytes.empty()) {
        bool checksumValid = verifyChecksum(bytes);
        if (checksumValid) {
            cout << "Data integrity verified" << endl;
        }
        else {
            cout << "Warning: Data integrity check failed" << endl;
            // 即使校验失败，也返回数据供分析
        }
    }

    return bytes;
}

// 简单的图像预处理
Mat preprocessImage(const Mat& input) {
    Mat processed;

    // 转换为灰度图
    if (input.channels() == 3) {
        cvtColor(input, processed, COLOR_BGR2GRAY);
    }
    else {
        processed = input.clone();
    }

    // 确保是二值图像
    threshold(processed, processed, 128, 255, THRESH_BINARY);

    return processed;
}

// 自动检测二维码区域
bool detectQRCode(const Mat& input, Mat& outputQR) {
    // 简单的实现：假设输入已经是正确的二维码图像
    // 在实际应用中，这里应该添加二维码检测和定位逻辑

    Mat processed = preprocessImage(input);

    // 确保图像是正方形的
    //int size = min(processed.rows, processed.cols);
    //if (size <= 0) {
    //    return false;
    //}

    //// 裁剪为正方形
    //Rect roi(0, 0, size, size);
    //outputQR = processed(roi).clone();
    outputQR = processed.clone();

    // 检查图像大小是否合理
    if (outputQR.rows % MODULE_SIZE != 0) {
        cout << "Warning: Image size may not match module size" << endl;
    }

    return true;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        cout << "Usage: decode <input_png> <output_bin> <validity_bin>\n";
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = argv[2];
    string validityFile = argv[3];

    // 读取输入图像
    Mat inputImage = imread(inputFile, IMREAD_GRAYSCALE);
    if (inputImage.empty()) {
        cerr << "Cannot open image: " << inputFile << endl;
        return 1;
    }

    cout << "Input image size: " << inputImage.cols << "x" << inputImage.rows << endl;

    // 检测并提取二维码
    Mat qrImage;
    if (!detectQRCode(inputImage, qrImage)) {
        cerr << "Failed to process QR code image" << endl;
        return 1;
    }

    cout << "QR image size: " << qrImage.rows << "x" << qrImage.cols << endl;

    vector<uint8_t> validity;
    vector<uint8_t> decodedData = decodeQRCode(qrImage, validity);

    if (decodedData.empty()) {
        cerr << "Error: No data decoded!" << endl;
        return 1;
    }

    writeBinaryFile(outputFile, decodedData);
    writeBinaryFile(validityFile, validity);

    cout << "Decoding completed successfully!" << endl;
    cout << "Output files: " << outputFile << ", " << validityFile << endl;
    cout << "Decoded data size: " << decodedData.size() << " bytes" << endl;

    return 0;
}