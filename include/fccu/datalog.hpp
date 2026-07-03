// Buffered CSV logger. RAII: the file closes (and flushes) on destruction.
#pragma once

#include <filesystem>
#include <fstream>
#include <string>

namespace fccu {

class CsvLogger {
public:
    CsvLogger(const std::filesystem::path& path, const std::string& header) {
        std::filesystem::create_directories(path.parent_path());
        file_.open(path);
        file_ << header << '\n';
    }

    bool ok() const { return file_.is_open() && file_.good(); }

    // flush per row: at 10 Hz the cost is nothing, and a Ctrl+C killed
    // process loses no data - the end of the log is where the fault lives
    void row(const std::string& line) {
        file_ << line << '\n';
        file_.flush();
    }

private:
    std::ofstream file_;
};

} // namespace fccu
