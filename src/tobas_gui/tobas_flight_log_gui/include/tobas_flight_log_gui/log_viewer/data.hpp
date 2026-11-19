// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDebug>
#include <QVector>

#include <tobas_path_tools/join.hpp>

#include "./message_decoder.hpp"

namespace tobas::gui::log
{
class DataI
{
public:
  explicit DataI(const std::string& _topic) : topic_(path::join('/', _topic))
  {
  }

  /** Ensure `std::unique_ptr<DataI>` invokes the derived destructor to release values and caches. */
  virtual ~DataI() = default;

  const std::string& getTopic() const
  {
    return topic_;
  }

  /** Clear plotted values and cached messages. */
  virtual void clear() = 0;

  /** Clear plotted values and keep cached messages within the new window. */
  virtual void prepareWindow(rcutils_time_point_value_t _start, rcutils_time_point_value_t _stop) = 0;

  virtual bool decode(rcutils_time_point_value_t _cur_time, const std::shared_ptr<rcutils_uint8_array_t>& _ser_data) = 0;

private:
  const std::string topic_;
};

template <typename MsgType>
class Data : public DataI
{
public:
  using DataI::DataI;

  void clear() override
  {
    values_.clear();
    decoder_.clearCache();
  }

  void prepareWindow(rcutils_time_point_value_t _start, rcutils_time_point_value_t _stop) override
  {
    values_.clear();
    decoder_.retainWindow(_start, _stop);
  }

  bool decode(rcutils_time_point_value_t _cur_time, const std::shared_ptr<rcutils_uint8_array_t>& _ser_data) override
  {
    const auto decoded = decoder_.decode(_cur_time, _ser_data);
    if (!decoded) {
      qWarning() << decoded.error().c_str();
      return false;
    }

    values_.append(**decoded);
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
}  // namespace tobas::gui::log
