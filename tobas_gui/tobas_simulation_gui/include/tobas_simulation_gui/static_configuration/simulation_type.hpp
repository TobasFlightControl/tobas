#pragma once

#include <QButtonGroup>
#include <QCheckBox>

namespace gui
{
namespace sim
{
class SimulationTypeWidget : public QWidget
{
  Q_OBJECT

  using self = SimulationTypeWidget;
  using super = QWidget;

public:
  enum type_t
  {
    SITL,
    HITL,
  };

  explicit SimulationTypeWidget();

  type_t simulationType() const;

private:
  QButtonGroup* ckb_group_;

  QCheckBox* sitl_ckb_;
  QCheckBox* hitl_ckb_;
};
}  // namespace sim
}  // namespace gui
