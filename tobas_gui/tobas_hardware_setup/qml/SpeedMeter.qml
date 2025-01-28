import QtQuick 2.15
import QtQuick.Extras 1.4
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: rectangle

  // Gauge: https://doc.qt.io/qt-5/qml-qtquick-extras-gauge.html
  Gauge {
    id: gauge
    objectName: "gauge"
    anchors.fill: parent // 親ウィジェットの大きさに自動で合わせる

    style: GaugeStyle {
      valueBar: Rectangle {
        color: "#e34c22"
        implicitWidth: gauge.width // 余白を埋める
      }
    }
  }

  // 関数呼び出し用シグナル
  signal setMaximumValue(double value)
  signal setMinimumValue(double value)
  signal setTickmarkStepSize(double value)
  signal setValue(double value)

  Component.onCompleted: {
    setMaximumValue.connect(onSetMaximumValue);
    setMinimumValue.connect(onSetMinimumValue);
    setTickmarkStepSize.connect(onSetTickmarkStepSize);
    setValue.connect(onSetValue);
  }

  function onSetMaximumValue(value)
  {
    gauge.maximumValue = value;
  }

  function onSetMinimumValue(value)
  {
    gauge.minimumValue = value;
  }

  function onSetTickmarkStepSize(value)
  {
    gauge.tickmarkStepSize = value;
  }

  function onSetValue(value)
  {
    gauge.value = value;
  }
}
