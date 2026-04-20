#include <iostream>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <sstream>
#include <cpp-can-parser/CANDatabase.h>
#include <generator.hpp>

static std::string calculateHash(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return "unknown";
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    size_t hash = std::hash<std::string>{}(content);
    std::stringstream ss;
    ss << std::hex << hash;
    return ss.str();
}

static std::string getDateString() {
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input.dbc> <output-dir>\n";
        return 1;
    }

    std::filesystem::path dbc_path   = argv[1];
    std::filesystem::path output_dir = argv[2];

    if (!std::filesystem::exists(dbc_path)) {
        std::cerr << "Error: " << dbc_path << " not found\n";
        return 1;
    }
    if (!std::filesystem::is_directory(output_dir)) {
        std::cerr << "Error: " << output_dir << " is not a directory\n";
        return 1;
    }

    CppCAN::CANDatabase db;
    try {
        db = CppCAN::CANDatabase::fromFile(dbc_path.string());
    } catch (const CppCAN::CANDatabaseException& e) {
        std::cerr << "Error parsing DBC: " << e.what() << "\n";
        return 1;
    }

    std::string hash = calculateHash(dbc_path.string());
    std::string date = getDateString();
    std::string code = generate(db, dbc_path.filename().string(), hash, date);

    auto output_file = output_dir / "UMDLoopCANProtocol.hpp";
    std::ofstream out(output_file);
    if (!out) {
        std::cerr << "Error: cannot write " << output_file << "\n";
        return 1;
    }
    out << code;
    std::cout << "Generated " << output_file << "\n";
    return 0;
}
