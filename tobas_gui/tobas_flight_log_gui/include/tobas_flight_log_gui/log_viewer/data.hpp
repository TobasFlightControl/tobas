// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDebug>
#include <QVector>

#include <tobas_path_tools/join.hpp>

#include "./message_decoder.hpp"

namespace tobas
{
namespace gui
{
namespace log
{
class DataI
{
public:
  explicit DataI(const std::string& topic) : topic_(path::join('/', topic))
  {
  }

  const std::string& getTopic() const
  {
    return topic_;
  }

  virtual void clearValues() = 0;
  virtual void clearCache() = 0;
  virtual bool decode(rcutils_time_point_value_t cur_time, const std::shared_ptr<rcutils_uint8_array_t>& ser_data) = 0;

private:
  const std::string topic_;
};

template <typename MsgType>
class Data : public DataI
{
public:
  using DataI::DataI;

  void clearValues() override
  {
    values_.clear();
  }

  void clearCache() override
  {
    decoder_.clearCache();
  }

  bool decode(rcutils_time_point_value_t cur_time, const std::shared_ptr<rcutils_uint8_array_t>& ser_data) override
  {
    try {
      values_.append(decoder_.decode(cur_time, ser_data));
    }
    catch (const std::exception& e) {
      qWarning() << e.what();
      return false;
    }

    return true;
  }

  const QVector<MsgType>& getValues() const
  {
    return values_;
  }

private:
  QVector<MsgType> values_;
  MessageDecoderCache<MsgType> decoder_;
};
}  // namespace log
}  // namespace gui
}  // namespace tobas
