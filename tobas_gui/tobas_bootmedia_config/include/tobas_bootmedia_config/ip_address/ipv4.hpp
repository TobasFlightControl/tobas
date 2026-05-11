#pragma once

#include <expected>

#include <tobas_qt_tools/widgets/combo_box.hpp>
#include <tobas_qt_tools/widgets/ipv4_edit.hpp>

#include "./base.hpp"

namespace tobas
{
namespace gui
{
namespace bm
{
class IPv4Widget : public BaseNetworkWidget
{
  Q_OBJECT

  using self = IPv4Widget;
  using super = BaseNetworkWidget;

  /* Method index */
  enum Method : int
  {
    kAutomatic = 0,
    kManual = 1,
  };

  /* Prefix length index */
  enum PrefixLength : int
  {
    kSlash8 = 0,
    kSlash16 = 1,
    kSlash24 = 2,
    kSlash25 = 3,
    kSlash26 = 4,
    kSlash27 = 5,
    kSlash28 = 6,
    kSlash29 = 7,
    kSlash30 = 8,
    kSlash32 = 9,
  };

public:
  explicit IPv4Widget(const QString& name);

  QString name() const override;

  void reset() override;

  bool load(const Network& src) override;
  Network dump() const override;

private:
  const QString name_;

  qt::ComboBox* method_;
  qt::ComboBox* prefix_;
  qt::IPv4Edit* address_;
  qt::IPv4Edit* gateway_;

  void enableManualFields(bool enabled);

  static std::expected<PrefixLength, QString> prefixValueToIndex(uint8_t value);
  static uint8_t prefixIndexToValue(PrefixLength index);

private Q_SLOTS:
  void onMethodChanged(int index);
};
}  // namespace bm
}  // namespace gui
}  // namespace tobas
