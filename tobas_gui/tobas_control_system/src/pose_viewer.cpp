#include "tobas_control_system/pose_viewer.hpp"

#include <QDebug>
#include <QPainter>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
PoseViewerWidget::PoseViewerWidget(const RosQtBridge& bridge)
{
  reset();

  connect(&bridge, &RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);
}

void PoseViewerWidget::reset()
{
  roll_ = 0.;
  pitch_ = 0.;
  yaw_ = 0.;

  update();
}

void PoseViewerWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  drawGround(painter);
  drawSky(painter);
  drawRoll(painter);
  drawPitch(painter);
  drawYaw(painter);

  addGradation(painter);
}

void PoseViewerWidget::scale(QPainter& painter, bool keep_aspect)
{
  if (keep_aspect) {
    const auto scale = static_cast<double>(std::min(width(), height())) / kOriginalSize;
    painter.scale(scale, scale);
  }
  else {
    const auto x_scale = static_cast<double>(width()) / kOriginalSize;
    const auto y_scale = static_cast<double>(height()) / kOriginalSize;
    painter.scale(x_scale, y_scale);
  }
}

void PoseViewerWidget::drawGround(QPainter& painter)
{
  painter.fillRect(rect(), QColor(169, 110, 45));
}

void PoseViewerWidget::drawSky(QPainter& painter)
{
  const auto w = width();
  const auto h = height();

  // 描画領域の四隅の点
  const QPoint OO(0, 0);
  const QPoint WO(w, 0);
  const QPoint OH(0, h);
  const QPoint WH(w, h);

  // 縦の画角に対するピッチ角の割合
  const auto r = pitch_ / (kPitchAngleOfView / 2);

  // 空の領域を構成する点群を作成 (memo: 3-43)
  QVector<QPoint> points;
  if (roll_ == 0.) {
    const auto y = (h / 2) * (1 - r);
    if (0. <= y && y <= h) {
      const QPoint OY(0, y);
      const QPoint WY(w, y);
      points = { OO, WO, WY, OY };
    }
    else {
      points = {};
    }
  }
  else {
    // 直線の方程式: y = ax + b
    const auto sin_phi = sin(roll_);
    const auto cos_phi = cos(roll_);
    const auto tan_phi = tan(roll_);
    const auto a = -tan_phi;
    const auto b = (tan_phi / 2) * (w - h * r * sin_phi) + (h / 2) * (1 - r * cos_phi);

    // 直線と描画領域の外辺の交点
    const QPoint XO(-b / a, 0);       // 直線と y = 0 の交点
    const QPoint XH((h - b) / a, h);  // 直線と y = h の交点
    const QPoint OY(0, b);            // 直線と x = 0 の交点
    const QPoint WY(w, a * w + b);    // 直線と x = w の交点

    // 描画領域の四隅がそれぞれ空領域に含まれるかどうかを判定
    const auto OO_sky = isSky(OO, a, b);
    const auto WO_sky = isSky(WO, a, b);
    const auto OH_sky = isSky(OH, a, b);
    const auto WH_sky = isSky(WH, a, b);

    if (!OO_sky && !WO_sky && !OH_sky && !WH_sky)  // 0000
    {
      points = {};
    }
    else if (!OO_sky && !WO_sky && !OH_sky && WH_sky)  // 0001
    {
      points = { WH, XH, WY };
    }
    else if (!OO_sky && !WO_sky && OH_sky && !WH_sky)  // 0010
    {
      points = { OH, XH, OY };
    }
    else if (!OO_sky && !WO_sky && OH_sky && WH_sky)  // 0011
    {
      points = { OH, WH, WY, OY };
    }
    else if (!OO_sky && WO_sky && !OH_sky && !WH_sky)  // 0100
    {
      points = { WO, XO, WY };
    }
    else if (!OO_sky && WO_sky && !OH_sky && WH_sky)  // 0101
    {
      points = { WO, WH, XH, XO };
    }
    else if (!OO_sky && WO_sky && OH_sky && WH_sky)  // 0111
    {
      points = { WO, WH, OH, OY, XO };
    }
    else if (OO_sky && !WO_sky && !OH_sky && !WH_sky)  // 1000
    {
      points = { OO, XO, OY };
    }
    else if (OO_sky && !WO_sky && OH_sky && !WH_sky)  // 1010
    {
      points = { OO, OH, XH, XO };
    }
    else if (OO_sky && !WO_sky && OH_sky && WH_sky)  // 1011
    {
      points = { OO, OH, WH, WY, XO };
    }
    else if (OO_sky && WO_sky && !OH_sky && !WH_sky)  // 1100
    {
      points = { OO, WO, WY, OY };
    }
    else if (OO_sky && WO_sky && !OH_sky && WH_sky)  // 1101
    {
      points = { OO, WO, WH, XH, OY };
    }
    else if (OO_sky && WO_sky && OH_sky && !WH_sky)  // 1110
    {
      points = { WO, OO, OH, XH, WY };
    }
    else if (OO_sky && WO_sky && OH_sky && WH_sky)  // 1111
    {
      points = { OO, WO, WH, OH };
    }
    else {
      qWarning() << "Impossible ground-sky pattern.";
      return;
    }
  }

  // 描画
  QPolygon polygon(points);
  painter.save();
  painter.setBrush(QColor(36, 139, 255));
  painter.drawPolygon(polygon);
  painter.restore();
}

void PoseViewerWidget::drawRoll(QPainter& painter)
{
  painter.save();

  // 機体から見た円の中心に移動
  painter.translate(width() / 2, height() / 2);
  painter.rotate(-st::rad2deg(roll_));

  // ウィジェットの大きさに合わせてスケーリング
  scale(painter, true);

  // 円を描画
  painter.setPen(QPen(Qt::white, kLineWidth));
  painter.drawEllipse(QPoint(0, 0), kRollRadius, kRollRadius);

  // 各値を描画
  const auto outer_radius = kRollRadius + kRollTickLength;
  const auto text_radius = outer_radius + 20;
  for (int deg = 0; deg < 360; deg += kScaleInterval) {
    // 目盛りを描画
    painter.drawLine(0, -kRollRadius, 0, -outer_radius);

    // 数字を描画
    painter.drawText(-10, -text_radius, std::format("{}°", math::wrap(deg, 180)).c_str());

    // 目盛りの間隔だけ進める
    painter.rotate(kScaleInterval);
  }

  painter.restore();

  // 現在の位置に目印を描く
  painter.save();
  painter.translate(width() / 2, height() / 2);
  scale(painter, true);
  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(0, -kRollRadius - kRollTickLength * 2, 0, -kRollRadius);
  painter.restore();
}

void PoseViewerWidget::drawPitch(QPainter& painter)
{
  painter.save();

  // 機体から見た中心位置に移動
  painter.translate(width() / 2, height() / 2);
  painter.rotate(-st::rad2deg(roll_));

  // ウィジェットの大きさに合わせてスケーリング
  scale(painter, true);

  // 描画する値の範囲を決める
  const auto pitch_deg = st::rad2deg(pitch_);
  const auto pitch_min = math::floor(pitch_deg - kPitchVisualRange, kScaleInterval);
  const auto pitch_max = math::ceil(pitch_deg + kPitchVisualRange, kScaleInterval);

  // 初期位置に移動
  painter.translate(0, pitchToHeight(st::deg2rad(pitch_min - pitch_deg)));

  // 各値を描画
  const auto line_half = kPitchLineLength / 2;
  const auto text_x = -line_half - 30;
  const auto text_y = 5;
  const auto y_interval = pitchToHeight(st::deg2rad(kScaleInterval));
  painter.setPen(QPen(Qt::white, kLineWidth));
  for (int deg = pitch_min; deg <= pitch_max; deg += kScaleInterval) {
    // 目盛りを描画
    painter.drawLine(-line_half, 0, line_half, 0);

    // 数字を描画
    painter.drawText(text_x, text_y, std::format("{}°", math::wrap(deg, 180)).c_str());

    // 目盛りの間隔だけ進める
    painter.translate(0, y_interval);
  }

  painter.restore();

  // 現在の位置に目印を描く
  painter.save();
  painter.translate(width() / 2, height() / 2);
  painter.rotate(-st::rad2deg(roll_));
  scale(painter, true);
  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(-line_half, 0, line_half, 0);
  painter.restore();
}

void PoseViewerWidget::drawYaw(QPainter& painter)
{
  painter.save();

  // ウィジェットの大きさに合わせてスケーリング
  scale(painter, false);

  // 中心位置に移動
  const auto beta = kYawAngleOfView / 2;  // [rad]
  painter.translate(yawToWidth(beta), kYawLineY);

  // 数直線を描画
  painter.setPen(QPen(Qt::white, kLineWidth));
  painter.drawLine(-kOriginalSize / 2, 0, kOriginalSize / 2, 0);

  // 描画する値の範囲を決める
  const auto yaw_deg = st::rad2deg(yaw_);
  const auto yaw_min = math::floor(st::rad2deg(yaw_ - beta), kScaleInterval);
  const auto yaw_max = math::ceil(st::rad2deg(yaw_ + beta), kScaleInterval);

  // 初期位置に移動
  painter.translate(yawToWidth(st::deg2rad(yaw_deg - yaw_min)), 0);

  // 各値を描画
  const auto text_x = -10;
  const auto text_y = -kYawTickLength - 20;
  const auto x_interval = yawToWidth(st::deg2rad(kScaleInterval));
  for (int deg = yaw_min; deg <= yaw_max; deg += kScaleInterval) {
    // 目盛りを描画
    painter.drawLine(0, 0, 0, -kYawTickLength);

    // 数字を描画
    painter.drawText(text_x, text_y, std::format("{}°", math::wrap(deg, 180)).c_str());

    // 目盛りの間隔だけ進める
    painter.translate(-x_interval, 0);
  }

  painter.restore();

  // 現在の位置に目印を描く
  painter.save();
  scale(painter, false);
  painter.translate(yawToWidth(beta), kYawLineY);
  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(0, 0, 0, -kYawTickLength * 2);
  painter.restore();
}

void PoseViewerWidget::addGradation(QPainter& painter)
{
  painter.save();

  // 中央を明るくするためのグラデーション設定
  QRadialGradient grad(getCenter(), width() / 2);

  grad.setColorAt(0, QColor(255, 255, 255, 100));  // 中心は半透明の白
  grad.setColorAt(1, QColor(255, 255, 255, 0));    // 外側は透明

  // ブラシにグラデーションを設定
  painter.setBrush(grad);
  painter.setPen(Qt::NoPen);  // ペンなしで塗りつぶす

  // 明るさを重ねる
  painter.drawRect(0, 0, width(), height());  // ウィジェット全体に円形グラデーションを適用

  painter.restore();
}

bool PoseViewerWidget::isSky(const QPoint& p, double a, double b) const
{
  const auto left = p.y();
  const auto right = a * p.x() + b;

  // ロール角で場合分け．ロール角が90度を超えている場合は天地が逆転している．
  if (std::abs(roll_) < M_PI_2) {
    return left < right;
  }
  else {
    return left > right;
  }
}

double PoseViewerWidget::pitchToHeight(double pitch)
{
  return kOriginalSize * pitch / kPitchAngleOfView;
}

double PoseViewerWidget::yawToWidth(double yaw)
{
  return kOriginalSize * yaw / kYawAngleOfView;
}

void PoseViewerWidget::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  // 現在のオイラー角を更新
  odom->odom.odom.frame.M.getRPY(roll_, pitch_, yaw_);

  // 再描画
  update();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
