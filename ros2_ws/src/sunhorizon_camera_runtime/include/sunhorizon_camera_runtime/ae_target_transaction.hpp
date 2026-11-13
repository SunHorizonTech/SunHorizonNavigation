#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace sunhorizon_camera_runtime {
struct AeTargetRange {int minimum{}, maximum{}, step{};};
struct AeTargetDevice {
  std::function<bool()> target_read_supported, target_write_supported, ae_read_supported;
  std::function<int()> read_target;
  std::function<AeTargetRange()> range;
  std::function<bool()> read_ae;
  std::function<void(int)> write_target;
};

// One fixed SDK property (IR_BRIGHTNESS/target_mean_intensity). No exposure,
// gain, laser, AE or HDR setter is available through this interface.
class AeTargetTransaction {
public:
  AeTargetTransaction(std::filesystem::path directory, std::string serial, AeTargetDevice device)
  : directory_(std::move(directory)), serial_(std::move(serial)), device_(std::move(device)) {
    need(!serial_.empty() && serial_.size() <= 64, "unsafe_serial");
    for (const unsigned char c : serial_) need((c >= 'A' && c <= 'Z') ||
      (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9'), "unsafe_serial");
    need(directory_.is_absolute() && directory_.lexically_normal() == directory_, "unsafe_journal_directory");
    need(!std::filesystem::is_symlink(std::filesystem::symlink_status(directory_)), "journal_directory_symlink");
    if (!std::filesystem::exists(directory_)) {status_ = "journal_directory_absent_no_override";return;}
    need(std::filesystem::canonical(directory_) == directory_, "journal_directory_symlink");
    dir_fd_ = ::open(directory_.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);
    need(dir_fd_ >= 0, "journal_directory_open");
    try {
      struct stat st{};
      need(::fstat(dir_fd_, &st) == 0 && S_ISDIR(st.st_mode) && st.st_uid == ::geteuid() &&
        !(st.st_mode & (S_IWGRP | S_IWOTH)), "journal_directory_permissions");
      const auto lock = serial_ + ".lock";
      lock_fd_ = ::openat(dir_fd_, lock.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
      need(lock_fd_ >= 0, "journal_lock_open");
      need(::fstat(lock_fd_, &st) == 0 && S_ISREG(st.st_mode) && st.st_nlink == 1 &&
        st.st_uid == ::geteuid() && !(st.st_mode & (S_IRWXG | S_IRWXO)), "journal_lock_permissions");
      need(::flock(lock_fd_, LOCK_EX | LOCK_NB) == 0, "journal_transaction_busy");
      read_journal();
      if (active_) {recovered_ = true;restore();}
    } catch (...) {close_files();throw;}
  }
  ~AeTargetTransaction() noexcept {restore_noexcept();close_files();}
  AeTargetTransaction(const AeTargetTransaction &) = delete;
  AeTargetTransaction & operator=(const AeTargetTransaction &) = delete;

  void begin(int requested) {
    need(requested >= -2, "invalid_override");
    requested_parameter_ = requested;
    if (requested == -1) {if (!recovered_) status_ = "override_disabled";return;}
    need(dir_fd_ >= 0, "explicit_journal_directory_required");
    need(!active_, "transaction_already_active");
    support();
    need(read_ae_enabled(), "auto_exposure_not_enabled");
    range_ = device_.range();
    const auto range = range_;
    original_ = device_.read_target();
    valid_value(original_, range);
    if (requested == -2) {
      // Half the actual target, rounded down on the device's min+n*step grid.
      // Never clamp upward from below min or silently return the original.
      const auto half = std::int64_t(original_) / 2;
      need(half >= range.minimum, "half_target_below_actual_minimum");
      requested_ = int(range.minimum + ((half - range.minimum) / range.step) * range.step);
    } else {requested_ = requested;}
    valid_value(requested_, range);
    need(requested_ < original_, "override_must_lower_original_target");
    // The durable PREPARED record is committed before the first device write.
    // A crash after it is durable always causes restoration, even if the SDK
    // setter changed hardware and then threw or APPLIED was never persisted.
    persist("prepared");active_ = true;new_override_ = true;status_ = "prepared";
    try {
      device_.write_target(requested_);
      readback_ = device_.read_target();
      need(readback_ == requested_ && read_ae_enabled(), "override_readback_or_ae_changed");
      persist("applied");status_ = "applied_restore_pending";
    } catch (...) {restore_noexcept();throw;}
  }

  void restore() {
    if (!active_) return;
    support();range_ = device_.range();valid_value(original_, range_);
    device_.write_target(original_);
    restored_readback_ = device_.read_target();
    need(restored_readback_ == original_, "restore_target_readback_failed");
    // The experiment never changes AE; unexpected non-target state is not
    // silently repaired by another setter. Leave an unresolved journal instead.
    need(read_ae_enabled(), "restore_ae_no_longer_enabled");
    persist("restored");active_ = false;status_ = recovered_ && !new_override_ ? "recovered_before_capture" : "restored_verified";
  }
  bool restore_noexcept() noexcept {
    try {restore();return true;} catch (const std::exception & e) {
      status_ = "restore_failed";error_ = e.what();
      try {persist("restore_failed");} catch (...) {}
      return false;
    } catch (...) {status_ = "restore_failed";return false;}
  }
  std::vector<std::pair<std::string, std::string>> receipt() const {
    return {{"ae_target_property", "OB_PROP_IR_BRIGHTNESS_INT:184"},
      {"ae_target_journal", (directory_ / (serial_ + ".txn")).string()},
      {"ae_target_transaction_state", status_}, {"ae_target_original", std::to_string(original_)},
      {"ae_target_requested_parameter", std::to_string(requested_parameter_)},
      {"ae_target_requested", std::to_string(requested_)}, {"ae_target_readback", std::to_string(readback_)},
      {"ae_target_restored_readback", std::to_string(restored_readback_)},
      {"ae_target_startup_recovered", recovered_ ? "true" : "false"},
      {"ae_target_restore_pending", active_ ? "true" : "false"},
      {"ae_target_range_min", std::to_string(range_.minimum)},
      {"ae_target_range_max", std::to_string(range_.maximum)},
      {"ae_target_range_step", std::to_string(range_.step)},
      {"ae_target_property_read_write_verified", support_verified_ ? "true" : "false"},
      {"ae_target_ae_readback", ae_readback_ < 0 ? "unqueried_or_unsupported" : (ae_readback_ ? "true" : "false")},
      {"ae_target_error", error_}};
  }
private:
  static void need(bool value, const char * reason) {
    if (!value) throw std::runtime_error(std::string("ae_target_") + reason);
  }
  static void valid_value(int value, const AeTargetRange & range) {
    need(range.step > 0 && range.minimum >= 0 && range.minimum <= range.maximum &&
      value >= range.minimum && value <= range.maximum &&
      (std::int64_t(value) - range.minimum) % range.step == 0, "actual_range_or_step");
  }
  bool read_ae_enabled() {
    if (!device_.ae_read_supported()) {ae_readback_ = -1;return false;}
    ae_readback_ = device_.read_ae() ? 1 : 0;return ae_readback_ == 1;
  }
  void support() {need(device_.target_read_supported() && device_.target_write_supported(), "property_not_read_write");support_verified_ = true;}
  void read_journal() {
    const auto name = serial_ + ".txn";
    const int fd = ::openat(dir_fd_, name.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0) {need(errno == ENOENT, "journal_read_open");return;}
    std::string raw;
    try {
      struct stat st{};
      need(::fstat(fd, &st) == 0 && S_ISREG(st.st_mode) && st.st_nlink == 1 &&
        st.st_uid == ::geteuid() && !(st.st_mode & (S_IRWXG | S_IRWXO)) &&
        st.st_size > 0 && st.st_size <= 4096, "journal_file_permissions_or_size");
      char bytes[4096];const auto count = ::read(fd, bytes, sizeof(bytes));
      need(count == st.st_size, "journal_short_read");raw.assign(bytes, std::size_t(count));
    } catch (...) {::close(fd);throw;}
    ::close(fd);
    std::istringstream in(raw);std::string magic, serial, property, state, extra;
    int ae{};
    need(bool(in >> magic >> serial >> property >> state >> original_ >> requested_ >> ae) &&
      !(in >> extra) && magic == "AE_TARGET_V1" && serial == serial_ && property == "184" && ae == 1 &&
      original_ >= 0 && requested_ >= 0 && requested_ < original_ &&
      (state == "prepared" || state == "applied" || state == "restore_failed" || state == "restored"),
      "journal_corrupt_or_identity_mismatch");
    active_ = state != "restored";
    status_ = active_ ? "startup_restore_required" : "prior_restored_verified";
  }
  void persist(const std::string & state) {
    const std::string raw = "AE_TARGET_V1\n" + serial_ + "\n184\n" + state + "\n" +
      std::to_string(original_) + "\n" + std::to_string(requested_) + "\n1\n";
    const auto temp = serial_ + ".tmp." + std::to_string(::getpid());
    // Only a secure regular owned temporary file may be reused after an
    // interrupted write. The serial lock excludes any other transaction owner.
    const int fd = ::openat(dir_fd_, temp.c_str(), O_CREAT | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, 0600);
    need(fd >= 0, "journal_temporary_open");
    try {
      struct stat st{};
      need(::fstat(fd, &st) == 0 && S_ISREG(st.st_mode) && st.st_nlink == 1 &&
        st.st_uid == ::geteuid() && !(st.st_mode & (S_IRWXG | S_IRWXO)), "journal_temporary_permissions");
      need(::ftruncate(fd, 0) == 0, "journal_truncate");
      std::size_t at = 0;
      while (at < raw.size()) {
        const auto count = ::write(fd, raw.data() + at, raw.size() - at);
        if (count < 0 && errno == EINTR) continue;
        need(count > 0, "journal_write");at += std::size_t(count);
      }
      need(::fsync(fd) == 0, "journal_file_fsync");
    } catch (...) {::close(fd);throw;}
    need(::close(fd) == 0, "journal_close");
    const auto destination = serial_ + ".txn";
    need(::renameat(dir_fd_, temp.c_str(), dir_fd_, destination.c_str()) == 0, "journal_atomic_rename");
    need(::fsync(dir_fd_) == 0, "journal_directory_fsync");
  }
  void close_files() noexcept {
    if (lock_fd_ >= 0) {::close(lock_fd_);lock_fd_ = -1;}
    if (dir_fd_ >= 0) {::close(dir_fd_);dir_fd_ = -1;}
  }
  std::filesystem::path directory_;
  std::string serial_, status_{"no_pending_transaction"}, error_;
  AeTargetDevice device_;
  int dir_fd_{-1}, lock_fd_{-1}, original_{-1}, requested_{-1}, requested_parameter_{-1}, readback_{-1}, restored_readback_{-1}, ae_readback_{-1};
  AeTargetRange range_;
  bool active_{false}, recovered_{false}, new_override_{false}, support_verified_{false};
};
}  // namespace sunhorizon_camera_runtime
