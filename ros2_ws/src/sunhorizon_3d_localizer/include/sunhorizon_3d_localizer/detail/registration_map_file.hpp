#pragma once
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <openssl/evp.h>

namespace sunhorizon_3d_localizer::detail {
// Pin the actual file around the existing PCL load; no second map model.
struct RegistrationMapFile {
  std::filesystem::path path;
  std::uintmax_t size;
  std::filesystem::file_time_type modified;
  std::string sha256;
  struct stat file_identity{};
  explicit RegistrationMapFile(const std::string& filename)
    :path(std::filesystem::canonical(filename)),size(std::filesystem::file_size(path)),
     modified(std::filesystem::last_write_time(path)) {
    if(::stat(path.c_str(),&file_identity)!=0)throw std::runtime_error("ndt_map_file_identity_unavailable");
    std::ifstream stream(path,std::ios::binary);
    std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(),EVP_MD_CTX_free);
    if(!stream || !context || EVP_DigestInit_ex(context.get(),EVP_sha256(),nullptr)!=1)
      throw std::runtime_error("ndt_map_digest_initialization_failed");
    std::array<char,65536> buffer;
    while(stream) {
      stream.read(buffer.data(),buffer.size());
      if(stream.gcount() && EVP_DigestUpdate(context.get(),buffer.data(),stream.gcount())!=1)
        throw std::runtime_error("ndt_map_digest_failed");
    }
    unsigned char hash[EVP_MAX_MD_SIZE];unsigned int length=0;
    if(!stream.eof() || EVP_DigestFinal_ex(context.get(),hash,&length)!=1 || length!=32)
      throw std::runtime_error("ndt_map_digest_failed");
    std::ostringstream out;out<<std::hex<<std::setfill('0');
    for(unsigned int i=0;i<length;++i)out<<std::setw(2)<<unsigned(hash[i]);
    sha256=out.str();verify(filename);
  }
  void verify(const std::string& filename) const {
    struct stat current{};
    if(std::filesystem::canonical(filename)!=path || std::filesystem::file_size(path)!=size ||
        std::filesystem::last_write_time(path)!=modified || ::stat(path.c_str(),&current)!=0 ||
        current.st_dev!=file_identity.st_dev || current.st_ino!=file_identity.st_ino)
      throw std::runtime_error("ndt_registration_map_changed_during_load");
  }
};
} // namespace sunhorizon_3d_localizer::detail
