#include <iostream>
#include <fstream>
#include <random>
#include <vector>
#include <cstdint>

int main() {
    const std::string filename = "random_data.bin";
    const size_t max_size = 10 * 1024 * 1024; // 10MB����󳤶�

    // ��ʼ�������������
    std::random_device rd;
    std::mt19937 gen(rd());
    // ʹ��int��Ϊ�ֲ�ģ�����������0��255֮����������
    std::uniform_int_distribution<int> distrib(0, 255);

    // ��������ļ������Զ�����ģʽ��
    std::ofstream outfile(filename, std::ios::binary);
    if (!outfile) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return 1;
    }

    // ����������ݲ�д���ļ�
    const size_t buffer_size = 4096; // 4KB������
    std::vector<uint8_t> buffer(buffer_size);

    size_t total_written = 0;
    while (total_written < max_size) {
        // ��仺���� with ����ֽ�
        for (size_t i = 0; i < buffer_size; ++i) {
            // ��int���ת��Ϊuint8_t
            buffer[i] = static_cast<uint8_t>(distrib(gen));
        }

        // ���㱾��Ҫд����ֽ�����������ʣ��������ֽ�����
        size_t bytes_to_write = std::min(buffer_size, max_size - total_written);

        // ������������д���ļ�
        outfile.write(reinterpret_cast<const char*>(buffer.data()), bytes_to_write);
        total_written += bytes_to_write;
    }

    outfile.close();

    std::cout << "Successfully generated random binary file: " << filename << std::endl;
    std::cout << "File size: " << total_written << " bytes ("
        << total_written / (1024.0 * 1024.0) << " MB)" << std::endl;

    return 0;
}