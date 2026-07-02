// Buffered CSV logger. RAII: the file closes (and flushes) on destruction.
#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace fccu {

class CsvLogger {
public:
    static constexpr int FLUSH_EVERY = 50;

    CsvLogger(const std::filesystem::path& path, const std::string& header) {
        std::filesystem::create_directories(path.parent_path());
        file_.open(path);
        file_ << header << '\n';
    }

    void row(const std::string& line) {
        file_ << line << '\n';
        if (++pending_ >= FLUSH_EVERY) {
            file_.flush();
            pending_ = 0;
        }
    }

private:
    std::ofstream file_;
    int pending_ = 0;
};

} // namespace fccu
