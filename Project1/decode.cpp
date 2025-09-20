#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <windows.h>
#include <direct.h> // 用于创建目录
#include <cstdint>

using namespace std;

// BMP文件头结构
#pragma pack(push, 1)
struct BMPHeader {
    uint16_t signature; // "BM"
    uint32_t fileSize;
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t dataOffset;
};

struct BMPInfoHeader {
    uint32_t headerSize;
    int32_t width;
    int32_t height;
    uint16_t planes;
    uint16_t bitsPerPixel;
    uint32_t compression;
    uint32_t imageSize;
    int32_t xPixelsPerMeter;
    int32_t yPixelsPerMeter;
    uint32_t colorsUsed;
    uint32_t colorsImportant;
};
#pragma pack(pop)

// 从BMP图像中提取单个比特信息
bool extract_bit_from_bmp(const string& filename, bool& is_valid) {
    ifstream bmp_file(filename, ios::binary);
    if (!bmp_file) {
        cerr << "无法打开BMP文件: " << filename << endl;
        is_valid = false;
        return false;
    }

    // 读取BMP头
    BMPHeader bmpHeader;
    BMPInfoHeader bmpInfoHeader;

    bmp_file.read(reinterpret_cast<char*>(&bmpHeader), sizeof(BMPHeader));
    bmp_file.read(reinterpret_cast<char*>(&bmpInfoHeader), sizeof(BMPInfoHeader));

    // 检查是否是有效的BMP文件
    if (bmpHeader.signature != 0x4D42) {
        cerr << "无效的BMP文件: " << filename << endl;
        is_valid = false;
        return false;
    }

    // 移动到像素数据
    bmp_file.seekg(bmpHeader.dataOffset, ios::beg);

    int width = bmpInfoHeader.width;
    int height = bmpInfoHeader.height;
    int rowSize = (width * 3 + 3) & ~3; // 每行字节数（需要4字节对齐）

    // 读取像素数据
    vector<uint8_t> pixelData(rowSize * height);
    bmp_file.read(reinterpret_cast<char*>(pixelData.data()), rowSize * height);
    bmp_file.close();

    // 计算整帧的平均亮度
    long total_brightness = 0;
    int total_pixels = width * height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int pos = y * rowSize + x * 3;
            uint8_t b = pixelData[pos];
            uint8_t g = pixelData[pos + 1];
            uint8_t r = pixelData[pos + 2];

            total_brightness += (r + g + b) / 3;
        }
    }

    int avg_brightness = total_brightness / total_pixels;

    // 判断帧的有效性：如果平均亮度接近纯黑或纯白，则认为有效
    is_valid = (avg_brightness < 10) || (avg_brightness > 245);

    // 判断比特值：如果平均亮度大于128，认为是1，否则为0
    return avg_brightness > 128;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "用法: " << argv[0] << " <输入视频> <输出文件> <有效性文件>" << endl;
        return 1;
    }

    string input_video = argv[1];
    string output_file = argv[2];
    string validity_file = argv[3];

    // 使用FFmpeg提取视频帧
    _mkdir("video_frames");
    string cmd = "ffmpeg -i " + input_video + " video_frames/frame_%d.bmp -y";
    system(cmd.c_str());

    // 处理每一帧
    vector<bool> all_bits;
    vector<bool> all_validity;
    int frame_count = 1;

    while (true) {
        stringstream ss;
        ss << "video_frames/frame_" << frame_count << ".bmp";

        ifstream test_file(ss.str());
        if (!test_file) break;
        test_file.close();

        bool is_valid;
        bool bit = extract_bit_from_bmp(ss.str(), is_valid);

        // 添加到总比特流
        all_bits.push_back(bit);
        all_validity.push_back(is_valid);

        frame_count++;
    }

    // 将比特流转换为字节
    ofstream out_file(output_file, ios::binary);
    ofstream valid_file(validity_file, ios::binary);

    for (size_t i = 0; i < all_bits.size(); i += 8) {
        if (i + 7 >= all_bits.size()) break;

        // 组合8个比特为一个字节
        unsigned char byte = 0;
        unsigned char valid_byte = 0;

        for (int j = 0; j < 8; j++) {
            if (all_bits[i + j]) {
                byte |= (1 << (7 - j));
            }
            if (all_validity[i + j]) {
                valid_byte |= (1 << (7 - j));
            }
        }

        out_file.put(byte);
        valid_file.put(valid_byte);
    }

    out_file.close();
    valid_file.close();

    // 清理临时文件
    system("rmdir /s /q video_frames");

    cout << "解码完成。输出文件: " << output_file << endl;
    cout << "有效性文件: " << validity_file << endl;
    cout << "处理的帧数: " << frame_count - 1 << endl;

    return 0;
}