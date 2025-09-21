#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
#include <cmath>

using namespace cv;
using namespace std;

// 二维码编码器命名空间
namespace Code
{
    constexpr int BytesPerFrame = 1242;
    constexpr int FrameSize = 108;
    constexpr int FrameOutputRate = 10;
    constexpr int SafeAreaWidth = 2;
    constexpr int QrPointSize = 18;
    constexpr int SmallQrPointbias = 6;
    constexpr int RectAreaCount = 7;
    const Vec3b pixel[8] =
    {
        Vec3b(0,0,0), Vec3b(0,0,255), Vec3b(0,255,0), Vec3b(0,255,255),
        Vec3b(255,0,0), Vec3b(255,0,255), Vec3b(255,255,0), Vec3b(255,255,255)
    };
    const int lenlim[RectAreaCount] = { 138,144,648,144,144,16,8 };
    const int areapos[RectAreaCount][2][2] =
    {
        {{69,16},{QrPointSize + 3,SafeAreaWidth}},
        {{16,72},{SafeAreaWidth,QrPointSize}},
        {{72,72},{QrPointSize,QrPointSize}},
        {{72,16},{QrPointSize,FrameSize - QrPointSize}},
        {{16,72},{FrameSize - QrPointSize,QrPointSize}},
        {{8,16},{FrameSize - QrPointSize,FrameSize - QrPointSize}},
        {{8,8},{FrameSize - QrPointSize + 8,FrameSize - QrPointSize}}
    };

    enum color
    {
        Black = 0,
        White = 7
    };

    enum class FrameType
    {
        Start = 0,
        End = 1,
        StartAndEnd = 2,
        Normal = 3
    };

    Mat ScaleToDisSize(const Mat& src)
    {
        Mat dis;
        constexpr int FrameOutputSize = FrameSize * FrameOutputRate;
        dis = Mat(FrameOutputSize, FrameOutputSize, CV_8UC3);
        for (int i = 0; i < FrameOutputSize; ++i)
        {
            for (int j = 0; j < FrameOutputSize; ++j)
            {
                dis.at<Vec3b>(i, j) = src.at<Vec3b>(i / 10, j / 10);
            }
        }
        return dis;
    }

    uint16_t CalCheckCode(const unsigned char* info, int len, bool isStart, bool isEnd, uint16_t frameBase)
    {
        // 1. 原逻辑：计算一个初始的校验值（可选，你可以根据需求调整或移除）
        uint16_t originalChecksum = 0;
        int cutlen = (len / 2) * 2;
        for (int i = 0; i < cutlen; i += 2)
            originalChecksum ^= ((uint16_t)info[i] << 8) | info[i + 1];
        if (len & 1)
            originalChecksum ^= (uint16_t)info[cutlen] << 8;
        originalChecksum ^= len;
        originalChecksum ^= frameBase;
        uint16_t temp = (isStart << 1) + isEnd;
        originalChecksum ^= temp;

        // 2. 将 originalChecksum (16位数据) 编码为海明码 (16+5=21位，但我们只关心校验位)
        // 海明码校验位数量计算：2^r >= 16 + r + 1 => r=5
        uint32_t data = originalChecksum; // 16位数据

        // 确定校验位的位置（在21位海明码中的位置：1,2,4,8,16）
        // 初始化海明码，所有位设为0
        uint32_t hammingCode = 0;

        // 将数据位放入海明码中（跳过校验位的位置）
        int dataBitPos = 0;
        for (int i = 1; i <= 21; i++) {
            // 如果 i 不是 2 的幂次方，则是数据位
            if ((i & (i - 1)) != 0) {
                if (data & (1 << dataBitPos)) {
                    hammingCode |= (1 << (i - 1));
                }
                dataBitPos++;
            }
        }

        // 计算每个校验位的值
        for (int i = 0; i < 5; i++) {
            int parityBitPos = (1 << i) - 1; // 校验位在海明码中的位置（0-indexed: 0,1,3,7,15）
            uint32_t parity = 0;

            // 遍历海明码的所有位
            for (int j = 1; j <= 21; j++) {
                if (j & (1 << i)) { // 如果该位受当前校验位校验
                    if (hammingCode & (1 << (j - 1))) {
                        parity ^= 1;
                    }
                }
            }
            if (parity) {
                hammingCode |= (1 << parityBitPos);
            }
        }

        // 3. 提取海明码中的5个校验位（位于位置1,2,4,8,16）
        uint16_t checkCode = 0;
        checkCode |= ((hammingCode >> 0) & 1) << 0;  // 位1
        checkCode |= ((hammingCode >> 1) & 1) << 1;  // 位2
        checkCode |= ((hammingCode >> 3) & 1) << 2;  // 位4
        checkCode |= ((hammingCode >> 7) & 1) << 3;  // 位8
        checkCode |= ((hammingCode >> 15) & 1) << 4; // 位16

        return checkCode; // 返回5位校验码（存储在16位变量的低5位）
    }
    void BulidSafeArea(Mat& mat)
    {
        constexpr int pos[4][2][2] =
        {
            {{0,FrameSize},{0,SafeAreaWidth}},
            {{0,FrameSize},{FrameSize - SafeAreaWidth,FrameSize}},
            {{0, SafeAreaWidth },{0,FrameSize}},
            {{FrameSize - SafeAreaWidth,FrameSize},{0,FrameSize}}
        };
        for (int k = 0; k < 4; ++k)
            for (int i = pos[k][0][0]; i < pos[k][0][1]; ++i)
                for (int j = pos[k][1][0]; j < pos[k][1][1]; ++j)
                    mat.at<Vec3b>(i, j) = pixel[White];
    }

    void BulidQrPoint(Mat& mat)
    {
        // 绘制大二维码识别点
        constexpr int pointPos[4][2] =
        {
            {0,0},
            {0,FrameSize - QrPointSize},
            {FrameSize - QrPointSize,0}
        };
        const Vec3b vec3bBig[9] =
        {
            pixel[Black],
            pixel[Black],
            pixel[Black],
            pixel[White],
            pixel[White],
            pixel[Black],
            pixel[Black],
            pixel[White],
            pixel[White]
        };
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < QrPointSize; ++j)
                for (int k = 0; k < QrPointSize; ++k)
                    mat.at<Vec3b>(pointPos[i][0] + j, pointPos[i][1] + k) =
                    vec3bBig[(int)max(fabs(j - 8.5), fabs(k - 8.5))];

        // 绘制小二维码识别点
        constexpr int posCenter[2] = { FrameSize - SmallQrPointbias,FrameSize - SmallQrPointbias };
        const Vec3b vec3bsmall[5] =
        {
            pixel[Black],
            pixel[Black],
            pixel[White],
            pixel[Black],
            pixel[White],
        };
        for (int i = -4; i <= 4; ++i)
            for (int j = -4; j <= 4; ++j)
                mat.at<Vec3b>(posCenter[0] + i, posCenter[1] + j) =
                vec3bsmall[max(abs(i), abs(j))];
    }

    void BulidCheckCodeAndFrameNo(Mat& mat, uint16_t checkcode, uint16_t FrameNo)
    {
        for (int i = 0; i < 5; ++i)
        {
            mat.at<Vec3b>(QrPointSize + 1, SafeAreaWidth + i) = pixel[(checkcode & 1) ? 7 : 0];
            checkcode >>= 1;
        }
        for (int i = 0; i < 16; ++i)
        {
            mat.at<Vec3b>(QrPointSize + 2, SafeAreaWidth + i) = pixel[(FrameNo & 1) ? 7 : 0];
            FrameNo >>= 1;
        }
    }

    void BulidInfoRect(Mat& mat, const char* info, int len, int areaID)
    {
        const unsigned char* pos = (const unsigned char*)info;
        const unsigned char* end = pos + len;
        for (int i = 0; i < areapos[areaID][0][0]; ++i)
        {
            uint32_t outputCode = 0;
            for (int j = 0; j < areapos[areaID][0][1] / 8; ++j)
            {
                outputCode |= *pos++;
                for (int k = areapos[areaID][1][1]; k < areapos[areaID][1][1] + 8; ++k)
                {
                    mat.at<Vec3b>(i + areapos[areaID][1][0], j * 8 + k) = pixel[(outputCode & 1) ? 7 : 0];
                    outputCode >>= 1;
                }
                if (pos == end) break;
            }
            if (pos == end) break;
        }
    }

    void BulidFrameFlag(Mat& mat, FrameType frameType, int tailLen)
    {
        switch (frameType)
        {
        case FrameType::Start:
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth) = pixel[White];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 1) = pixel[White];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 2) = pixel[Black];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 3) = pixel[Black];
            break;
        case FrameType::End:
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth) = pixel[Black];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 1) = pixel[Black];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 2) = pixel[White];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 3) = pixel[White];
            break;
        case FrameType::StartAndEnd:
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth) = pixel[White];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 1) = pixel[White];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 2) = pixel[White];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 3) = pixel[White];
            break;
        default:
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth) = pixel[Black];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 1) = pixel[Black];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 2) = pixel[Black];
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + 3) = pixel[Black];
            break;
        }
        for (int i = 4; i < 16; ++i)
        {
            mat.at<Vec3b>(QrPointSize, SafeAreaWidth + i) = pixel[(tailLen & 1) ? 7 : 0];
            tailLen >>= 1;
        }
    }

    Mat CodeFrame(FrameType frameType, const char* info, int tailLen, int FrameNo)
    {
        Mat codeMat = Mat(FrameSize, FrameSize, CV_8UC3, Vec3d(255, 255, 255));
        if (frameType != FrameType::End && frameType != FrameType::StartAndEnd)
            tailLen = BytesPerFrame;
        BulidSafeArea(codeMat);
        BulidQrPoint(codeMat);

        int checkCode = CalCheckCode((const unsigned char*)info, tailLen,
            frameType == FrameType::Start || frameType == FrameType::StartAndEnd,
            frameType == FrameType::End || frameType == FrameType::StartAndEnd, FrameNo);
        BulidFrameFlag(codeMat, frameType, tailLen);
        BulidCheckCodeAndFrameNo(codeMat, checkCode, FrameNo % 65536);
        if (tailLen != BytesPerFrame)
            tailLen = BytesPerFrame;
        for (int i = 0; i < RectAreaCount && tailLen>0; ++i)
        {
            int lennow = std::min(tailLen, lenlim[i]);
            BulidInfoRect(codeMat, info, lennow, i);
            tailLen -= lennow;
            info += lennow;
        }
        return codeMat;
    }

    vector<Mat> GenerateQrFrames(const char* info, int len, int frameRate, int maxDurationMs)
    {
        vector<Mat> frames;
        int maxFrames = (maxDurationMs * frameRate) / 1000;

        if (maxFrames == 0) return frames;
        if (len <= 0) return frames;

        if (len <= BytesPerFrame)
        {
            unsigned char BUF[BytesPerFrame + 5];
            memcpy(BUF, info, sizeof(unsigned char) * len);
            for (int i = len; i < BytesPerFrame; ++i)
                BUF[i] = rand() % 256;

            Mat frame = CodeFrame(FrameType::StartAndEnd, (char*)BUF, len, 0);
            frames.push_back(ScaleToDisSize(frame));
        }
        else
        {
            int frameCount = 0;
            len -= BytesPerFrame;
            Mat frame = ScaleToDisSize(CodeFrame(FrameType::Start, info, len, frameCount++));
            frames.push_back(frame);

            while (len > 0 && frames.size() < maxFrames)
            {
                info += BytesPerFrame;

                if (len - BytesPerFrame > 0)
                {
                    if (frames.size() + 1 < maxFrames)
                        frame = ScaleToDisSize(CodeFrame(FrameType::Normal, info, BytesPerFrame, frameCount++));
                    else
                        frame = ScaleToDisSize(CodeFrame(FrameType::End, info, BytesPerFrame, frameCount++));
                }
                else
                {
                    unsigned char BUF[BytesPerFrame + 5];
                    memcpy(BUF, info, sizeof(unsigned char) * len);
                    for (int i = len; i < BytesPerFrame; ++i)
                        BUF[i] = rand() % 256;
                    frame = ScaleToDisSize(CodeFrame(FrameType::End, (char*)BUF, len, frameCount++));
                }

                frames.push_back(frame);
                len -= BytesPerFrame;
            };
        }

        return frames;
    }
}

int main(int argc, char* argv[])
{
    // 检查参数
    if (argc != 4)
    {
        cerr << "Usage: encode <input_file> <output_video> <duration_ms>" << endl;
        return 1;
    }

    string inputFile = argv[1];
    string outputVideo = argv[2];
    int durationMs = atoi(argv[3]);

    if (durationMs <= 0)
    {
        cerr << "Error: Duration must be a positive integer" << endl;
        return 1;
    }

    // 读取输入文件
    ifstream file(inputFile, ios::binary | ios::ate);
    if (!file.is_open())
    {
        cerr << "Error: Cannot open input file" << endl;
        return 1;
    }

    streamsize size = file.tellg();
    file.seekg(0, ios::beg);

    vector<char> buffer(size);
    if (!file.read(buffer.data(), size))
    {
        cerr << "Error: Cannot read input file" << endl;
        return 1;
    }
    file.close();

    // 设置随机种子
    srand(static_cast<unsigned int>(time(nullptr)));

    // 生成二维码帧
    int frameRate = 10; // 固定帧率
    vector<Mat> frames = Code::GenerateQrFrames(buffer.data(), size, frameRate, durationMs);

    if (frames.empty())
    {
        cerr << "Error: No frames generated" << endl;
        return 1;
    }

    // 创建视频写入器
    Size frameSize = frames[0].size();
    VideoWriter writer;
    writer.open(outputVideo, VideoWriter::fourcc('m', 'p', '4', 'v'), frameRate, frameSize);

    if (!writer.isOpened())
    {
        cerr << "Error: Could not open video writer" << endl;
        return 1;
    }

    // 写入帧到视频
    for (const auto& frame : frames)
    {
        writer.write(frame);
    }

    writer.release();
    cout << "Video encoded successfully: " << outputVideo << endl;
    cout << "Frames: " << frames.size() << ", Duration: " << (frames.size() * 1000 / frameRate) << "ms" << endl;

    return 0;
}
