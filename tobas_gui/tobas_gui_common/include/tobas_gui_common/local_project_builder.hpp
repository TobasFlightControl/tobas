#pragma once

#include <QThread>
#include <expected>

#include <tobas_colcon_cpp/core.hpp>

namespace gui
{
namespace cmn
{
class LocalProjectBuilder
{
public:
  explicit LocalProjectBuilder();

  bool build(const std::filesystem::path& proj_path);

  const std::string& errorMessage() const;

private:
  colcon::Colcon colcon_;
};

class LocalProjectBuilderThread : public QThread
{
  Q_OBJECT

Q_SIGNALS:
  void finished(bool success, const QString& message);

public:
  explicit LocalProjectBuilderThread(const std::filesystem::path& proj_path);

  void run() override;

private:
  const std::filesystem::path proj_path_;

  LocalProjectBuilder builder_;
};

/* Qtスレッドを止めずにローカルプロジェクトをビルドする． */
std::expected<void, QString> buildLocalProject(const std::filesystem::path& proj_path);
}  // namespace cmn
}  // namespace gui
