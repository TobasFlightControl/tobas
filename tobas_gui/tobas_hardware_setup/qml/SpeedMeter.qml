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
    maximumValue: 100
    minimumValue: 0
    tickmarkStepSize: computeTickmarkStepSize() // 他のメンバ変数に応じて自動調整
    value: 0

    function computeTickmarkStepSize()
    {
      var step = gauge.maximumValue / 10
      var units = [1000, 500, 200, 100, 50, 20, 10, 5, 2, 1]

      for (var i = 0; i < units.length; i++)
      {
        if (step >= units[i])
        {
          return Math.ceil(step / units[i]) * units[i];
        }
      }
    }

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
  signal setValue(double value)

  Component.onCompleted: {
    setMaximumValue.connect(onSetMaximumValue);
    setMinimumValue.connect(onSetMinimumValue);
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

  function onSetValue(value)
  {
    gauge.value = value;
  }
}
