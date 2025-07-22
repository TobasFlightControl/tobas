#pragma once

#include <QButtonGroup>
#include <QCheckBox>

namespace gui
{
namespace sim
{
enum LoopType
{
  SITL,
  HITL,
};

class LoopTypeWidget : public QWidget
{
  Q_OBJECT

  using self = LoopTypeWidget;
  using super = QWidget;

public:
  explicit LoopTypeWidget();

  LoopType loopType() const;

private:
  QButtonGroup* ckb_group_;

  QCheckBox* sitl_ckb_;
  QCheckBox* hitl_ckb_;
};
}  // namespace sim
}  // namespace gui
