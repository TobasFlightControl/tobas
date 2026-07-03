// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ssh_authkey/prettify.hpp"

namespace tobas
{
namespace ssh
{
namespace ak
{
namespace
{
std::string base64NoPad(const uint8_t* p, size_t n)
{
  static constexpr char T[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string out;
  out.reserve((n * 4 + 2) / 3);

  size_t i = 0;
  while (i + 3 <= n) {
    uint32_t v = (uint32_t(p[i]) << 16) | (uint32_t(p[i + 1]) << 8) | uint32_t(p[i + 2]);
    out.push_back(T[(v >> 18) & 63]);
    out.push_back(T[(v >> 12) & 63]);
    out.push_back(T[(v >> 6) & 63]);
    out.push_back(T[v & 63]);
    i += 3;
  }

  if (i < n) {
    uint32_t v = uint32_t(p[i]) << 16;
    if (i + 1 < n) {
      v |= uint32_t(p[i + 1]) << 8;
    }
    out.push_back(T[(v >> 18) & 63]);
    out.push_back(T[(v >> 12) & 63]);
    if (i + 1 < n) {
      out.push_back(T[(v >> 6) & 63]);  // Do not add padding.
    }
  }

  return out;
}
}  // namespace

std::expected<std::string, std::string> prettify(const Data& src)
{
  if (src.key_type == SSH_KEYTYPE_UNKNOWN) {
    return std::unexpected("SSH key type is unknown.");
  }

  if (!src.key) {
    return std::unexpected("SSH key is null.");
  }

  // Type name.
  const auto key_type_name = ssh_key_type_to_char(src.key_type);
  if (!key_type_name) {
    return std::unexpected("Failed to get SSH key type name.");
  }

  // SHA256 fingerprint, matching the default OpenSSH display.
  uint8_t* hash = nullptr;
  size_t hlen = 0;
  if (ssh_get_publickey_hash(src.key, SSH_PUBLICKEY_HASH_SHA256, &hash, &hlen) != SSH_OK || !hash || hlen == 0) {
    return std::unexpected("SHA256 is unavailable.");
  }

  const auto fp_sha256 = "SHA256:" + base64NoPad(hash, hlen);
  ssh_clean_pubkey_hash(&hash);

  return std::string(key_type_name) + " " + fp_sha256 + " " + src.comment;
}
}  // namespace ak
}  // namespace ssh
}  // namespace tobas
