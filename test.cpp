#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <cstdint>

int main() {
    const std::string filename = "random_data.bin";
    const size_t max_size = 10 * 1024 * 1024; // 10MB的最大长度

    // 初始化随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    // 使用int作为分布模板参数，生成0到255之间的随机整数
    std::uniform_int_distribution<int> distrib(0, 255);

    // 创建输出文件流，以二进制模式打开
    std::ofstream outfile(filename, std::ios::binary);
    if (!outfile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return 1;
    }

    // 生成随机数据并写入文件
    const size_t buffer_size = 4096; // 4KB缓冲区
    std::vector<uint8_t> buffer(buffer_size);

    size_t total_written = 0;
    while (total_written < max_size) {
        // 填充缓冲区 with 随机字节
        for (size_t i = 0; i < buffer_size; ++i) {
            // 将int结果转换为uint8_t
            buffer[i] = static_cast<uint8_t>(distrib(gen));
        }

        // 计算本次要写入的字节数（不超过剩余允许的字节数）
        size_t bytes_to_write = std::min(buffer_size, max_size - total_written);

        // 将缓冲区数据写入文件
        outfile.write(reinterpret_cast<const char*>(buffer.data()), bytes_to_write);
        total_written += bytes_to_write;
    }

    outfile.close();

    std::cout << "Successfully generated random binary file: " << filename << std::endl;
    std::cout << "File size: " << total_written << " bytes ("
        << total_written / (1024.0 * 1024.0) << " MB)" << std::endl;

    return 0;
}