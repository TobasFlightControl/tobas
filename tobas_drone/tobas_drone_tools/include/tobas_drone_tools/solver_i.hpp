#pragma once

#include <tobas_kdl/tree.hpp>
#include <tobas_drone_core/drone.hpp>

namespace tobas
{
class SolverI
{
protected:
  static constexpr char kErrorNotUpToDate[] = "Internal data structures not up to date with Tree";
  static constexpr char kErrorSizeMismatch[] = "The size of input doesn't match the internal state";
  static constexpr char kErrorUnknownErrorCode[] = "Unknown error code";
  static constexpr char kOutOfRange[] = "The requested index is out of range";

public:
  enum error_t : int
  {
    E_NO_ERROR = 0,  // エラーなし
    E_WARN = -1,     // 警告のみで処理続行
    E_ERROR = -2,    // 処理を停止すべき重大なエラー
  };

  virtual bool updateInternalDataStructures() = 0;

  inline const int& errorCode() const;
  inline const std::string& errorMessage() const;

protected:
  int error_code_ = E_NO_ERROR;
  std::string error_msg_;

  /* 引数のエラーコードの方が深刻な場合はエラーを更新し，現在のエラーコードを返す． */
  inline int updateError(const SolverI& arg);
};

inline const int& SolverI::errorCode() const
{
  return error_code_;
}

inline const std::string& SolverI::errorMessage() const
{
  return error_msg_;
}

inline int SolverI::updateError(const SolverI& arg)
{
  if (arg.errorCode() < error_code_)
  {
    error_code_ = arg.errorCode();
    error_msg_ = arg.errorMessage();
  }
  return error_code_;
}
}  // namespace tobas
