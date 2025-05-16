#pragma once

#include <memory>

#include "../utils/urdf_clone.hpp"

namespace gui
{
namespace urdf_builder
{
namespace view_model
{
/**
 * @brief URDFの要素クラスのラッパー．
 *
 * @tparam M URDFの要素クラス
 * @tparam Derived 派生クラス
 */
template <typename M, typename Derived>
class BaseViewModel
{
public:
  using ModelPtr = std::shared_ptr<M>;
  using DerivedPtr = std::shared_ptr<Derived>;

  explicit BaseViewModel(const ModelPtr& model) : model_(model)
  {
    if (!model_) {
      model_.reset(new M());
    }
  }

  const ModelPtr& model()
  {
    return model_;
  }

  DerivedPtr clone() const
  {
    return DerivedPtr(new Derived(utils::clone(model_)));
  }

  /* View Modelの内容をURDFモデルに反映させる． */
  virtual void sync() = 0;

protected:
  ModelPtr model_;
};
}  // namespace view_model
}  // namespace urdf_builder
}  // namespace gui
