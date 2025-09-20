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
    uint16_t signature = 0x4D42; // "BM"
    uint32_t fileSize;
    uint16_t reserved1 = 0;
    uint16_t reserved2 = 0;
    uint32_t dataOffset;
};

struct BMPInfoHeader {
    uint32_t headerSize = 40;
    int32_t width;
    int32_t height;
    uint16_t planes = 1;
    uint16_t bitsPerPixel = 24;
    uint32_t compression = 0;
    uint32_t imageSize;
    int32_t xPixelsPerMeter = 0;
    int32_t yPixelsPerMeter = 0;
    uint32_t colorsUsed = 0;
    uint32_t colorsImportant = 0;
};
#pragma pack(pop)

// 生成BMP图像文件（全黑或全白）
void generate_bmp(const string& filename, bool bit_value) {
    ofstream bmp_file(filename, ios::binary);
    if (!bmp_file) {
        cerr << "无法创建BMP文件: " << filename << endl;
        exit(1);
    }

    int width = 640;
    int height = 480;
    int rowSize = (width * 3 + 3) & ~3; // 每行字节数（需要4字节对齐）
    int imageSize = rowSize * height;

    // 设置BMP头
    BMPHeader bmpHeader;
    bmpHeader.fileSize = sizeof(BMPHeader) + sizeof(BMPInfoHeader) + imageSize;
    bmpHeader.dataOffset = sizeof(BMPHeader) + sizeof(BMPInfoHeader);

    BMPInfoHeader bmpInfoHeader;
    bmpInfoHeader.width = width;
    bmpInfoHeader.height = height;
    bmpInfoHeader.imageSize = imageSize;

    // 写入BMP头
    bmp_file.write(reinterpret_cast<char*>(&bmpHeader), sizeof(BMPHeader));
    bmp_file.write(reinterpret_cast<char*>(&bmpInfoHeader), sizeof(BMPInfoHeader));

    // 生成图像内容（全黑或全白）
    vector<uint8_t> pixelData(imageSize, 0);

    if (bit_value) {
        // 白色表示1
        for (int i = 0; i < imageSize; i++) {
            pixelData[i] = 255;
        }
    }
    // 否则保持黑色(0)

    // 写入像素数据
    bmp_file.write(reinterpret_cast<char*>(pixelData.data()), imageSize);
    bmp_file.close();
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        cerr << "用法: " << argv[0] << " <输入文件> <输出视频> <时长(毫秒)>" << endl;
        return 1;
    }

    string input_file = argv[1];
    string output_video = argv[2];
    int duration_ms = stoi(argv[3]);

    // 读取输入文件
    ifstream file(input_file, ios::binary);
    if (!file) {
        cerr << "无法打开输入文件: " << input_file << endl;
        return 1;
    }

    // 读取文件内容到比特流
    vector<bool> bits;
    char byte;
    while (file.get(byte)) {
        for (int i = 7; i >= 0; i--) {
            bits.push_back((byte >> i) & 1);
        }
    }
    file.close();

    // 计算帧数和每帧包含的比特数（每帧一个比特）
    double fps = 30.0; // 假设30fps
    double duration_sec = duration_ms / 1000.0;
    int total_frames = static_cast<int>(round(fps * duration_sec));

    // 如果比特数超过可传输的帧数，则截断
    if (bits.size() > total_frames) {
        bits.resize(total_frames);
        cout << "警告: 数据被截断，只传输前 " << total_frames << " 个比特" << endl;
    }

    // 创建临时目录存放帧
    _mkdir("frames");

    // 生成帧图像（每帧一个比特）
    for (int i = 0; i < bits.size(); i++) {
        stringstream ss;
        ss << "frames/frame_" << i << ".bmp";
        generate_bmp(ss.str(), bits[i]);
    }

    // 使用FFmpeg创建视频
    string cmd = "ffmpeg -r " + to_string(fps) +
                 " -i frames/frame_%d.bmp -c:v libx264 -pix_fmt yuv420p " +
                 output_video + " -y";
    system(cmd.c_str());

    // 清理临时文件
    system("rmdir /s /q frames");

    cout << "视频编码完成: " << output_video << endl;
    cout << "传输的比特数: " << bits.size() << endl;

    return 0;
}