#ifndef FAIRINO_HARDWARE__SERVO_J_DIAGNOSTICS_HPP_
#define FAIRINO_HARDWARE__SERVO_J_DIAGNOSTICS_HPP_

#include <cmath>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <ostream>
#include <string>

namespace fairino_hardware
{

inline void json_string(std::ostream & out, const std::string & value)
{
  static constexpr char hex[] = "0123456789abcdef";
  out << '"';
  for (const unsigned char ch : value) {
    if (ch == '"' || ch == '\\') {out << '\\' << ch;} else if (ch < 0x20) {
      out << "\\u00" << hex[ch >> 4] << hex[ch & 15];
    } else {out << ch;}
  }
  out << '"';
}

inline void json_number(std::ostream & out, double value)
{
  if (std::isfinite(value)) {out << value;} else {out << "null";}
}

template<class Values>
void json_six(std::ostream & out, const Values & values)
{
  out << '[';
  for (size_t index = 0; index < 6; ++index) {
    if (index) {out << ',';}
    json_number(out, values[index]);
  }
  out << ']';
}

// Used only by non-servo callbacks and after stream termination. Two bounded
// files retain recent controller feedback without unbounded Docker stdout.
class BoundedJsonlFile
{
public:
  explicit BoundedJsonlFile(std::string path, size_t maximum_bytes = 8 * 1024 * 1024)
  : path_(std::move(path)), maximum_bytes_(maximum_bytes) {}

  bool append(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (line.size() + 1 > maximum_bytes_) {return false;}
    std::error_code error;
    auto bytes = std::filesystem::file_size(path_, error);
    if (error) {bytes = 0;}
    if (bytes + line.size() + 1 > maximum_bytes_) {
      std::filesystem::remove(path_ + ".1", error);
      error.clear();
      std::filesystem::rename(path_, path_ + ".1", error);
      if (error) {return false;}
    }
    std::ofstream out(path_, std::ios::app);
    out << line << '\n';
    out.flush();
    return out.good();
  }

private:
  std::string path_;
  size_t maximum_bytes_;
  std::mutex mutex_;
};

}  // namespace fairino_hardware

#endif  // FAIRINO_HARDWARE__SERVO_J_DIAGNOSTICS_HPP_
