// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_crypt/crypt.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include <tobas_linux/error.hpp>

namespace ch = std::chrono;

namespace tobas
{
namespace crypt
{
namespace
{
/* Read `/etc/shadow` line by line. */
std::vector<std::string> readLines(const std::string& path)
{
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "Failed to open " << path << ": " << linux::strError() << std::endl;
    return {};
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(ifs, line)) {
    lines.push_back(line);
  }

  return lines;
}

/* Convert colon-separated text to an array. */
std::vector<std::string> splitShadow(const std::string& line)
{
  std::vector<std::string> fields;
  size_t start = 0;

  while (true) {
    const auto pos = line.find(':', start);
    if (pos == std::string::npos) {
      fields.push_back(line.substr(start));
      break;
    }

    fields.push_back(line.substr(start, pos - start));
    start = pos + 1;
  }

  return fields;
}

/* Convert an array back to colon-separated text. */
std::string joinShadow(const std::vector<std::string>& fields)
{
  std::ostringstream ss;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i) {
      ss << ':';
    }
    ss << fields[i];
  }
  return ss.str();
}

/* Safely overwrite a file. */
bool atomicOverwrite(const std::string& path, const std::string& content)
{
  // Save existing metadata.
  struct stat st;
  if (stat(path.c_str(), &st) < 0) {
    std::cerr << "stat failed on " + path + ": " << linux::strError() << std::endl;
    return false;
  }

  // Create a temporary file in the same directory.
  const auto dir = path.substr(0, path.find_last_of('/'));
  auto tmp = dir + "/.shadow.tmp.XXXXXX";
  std::vector<char> tmpc(tmp.begin(), tmp.end());
  tmpc.push_back('\0');

  auto fd = ::mkstemp(tmpc.data());
  if (fd < 0) {
    std::cerr << "mkstemp failed: " << linux::strError() << std::endl;
    return false;
  }
  tmp.assign(tmpc.data());

  // Match permissions and owner, overwriting with 0640 for safety.
  if (fchmod(fd, st.st_mode & 0640 ? st.st_mode : 0640) < 0) {
    std::cerr << "Failed to change mode." << std::endl;
    return false;
  }
  if (fchown(fd, st.st_uid, st.st_gid) < 0) {
    std::cerr << "Failed to change owner." << std::endl;
    return false;
  }

  // Write.
  const auto wr = ::write(fd, content.data(), content.size());
  if (wr != static_cast<ssize_t>(content.size())) {
    ::close(fd);
    ::unlink(tmp.c_str());
    std::cerr << "write failed" << std::endl;
    return false;
  }

  // Append a newline if missing.
  if (content.empty() || content.back() != '\n') {
    if (::write(fd, "\n", 1) != 1) {
      std::cerr << "write failed: " << linux::strError() << std::endl;
      return false;
    }
  }

  // Flush to disk.
  if (::fsync(fd) < 0) {
    ::close(fd);
    ::unlink(tmp.c_str());
    std::cerr << "fsync failed" << std::endl;
    return false;
  }
  ::close(fd);

  // Replace atomically.
  if (::rename(tmp.c_str(), path.c_str()) < 0) {
    ::unlink(tmp.c_str());
    std::cerr << "rename failed: " << linux::strError() << std::endl;
    return false;
  }

  return true;
}
}  // namespace

bool setShadowPassword(
  const std::string& _shadow_path,
  const std::string& _username,
  const std::string& _new_password,
  const Crypt& _crypt)
{
  auto lines = readLines(_shadow_path);
  if (lines.empty()) {
    return false;
  }

  // Generate hash.
  const auto hash = _crypt.crypt(_new_password);
  if (hash.empty()) {
    return false;
  }

  // Get the password change date as the number of days since the Unix epoch.
  const auto days = duration_cast<ch::hours>(ch::system_clock::now().time_since_epoch()).count() / 24;

  // Change only the user login password.
  bool found = false;
  for (auto& line : lines) {
    // Skip blank and comment lines.
    if (line.empty() || line[0] == '#') {
      continue;
    }

    auto fields = splitShadow(line);
    if (fields.empty()) {
      continue;
    }

    if (fields[0] == _username) {
      // Fill missing fields if there are too few.
      constexpr size_t kMinNumFields = 9;
      if (fields.size() < kMinNumFields) {
        fields.resize(kMinNumFields, "");
      }

      fields[1] = hash;                  // Hash.
      fields[2] = std::to_string(days);  // Last change date.
      line = joinShadow(fields);
      found = true;
      break;
    }
  }

  if (!found) {
    std::cerr << "user not found in shadow: " << _username << std::endl;
    return false;
  }

  // Assemble content.
  std::ostringstream out;
  for (size_t i = 0; i < lines.size(); ++i) {
    out << lines[i];
    if (i + 1 < lines.size()) {
      out << std::endl;
    }
  }

  // Safely overwrite the file.
  if (!atomicOverwrite(_shadow_path, out.str())) {
    return false;
  }

  return true;
}
}  // namespace crypt
}  // namespace tobas
