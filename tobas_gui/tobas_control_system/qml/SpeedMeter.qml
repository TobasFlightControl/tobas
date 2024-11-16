import QtQuick 2.15
import QtQuick.Extras 1.4
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: rectangle

  // CircularGauge: https://doc.qt.io/qt-5/qml-qtquick-extras-circulargauge.html
  CircularGauge {
    id: gauge
    anchors.fill: parent // 親ウィジェットの大きさに自動で合わせる

    // CircularGaugeStyle: https://doc.qt.io/qt-5/qml-qtquick-controls-styles-circulargaugestyle.html
    // 親のメンバが更新されたらスタイルのメンバも自動で更新される
    style: CircularGaugeStyle {
      tickmarkStepSize: computeTickmarkStepSize()

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
    }
  }

  Text {
    id: topText
    anchors.top: gauge.top
    anchors.horizontalCenter: gauge.horizontalCenter
    anchors.topMargin: gauge.height / 4
    text: ""
    color: "black"
    font.pixelSize: 20
  }

  Text {
    id: bottomText
    anchors.bottom: gauge.bottom
    anchors.horizontalCenter: gauge.horizontalCenter
    anchors.bottomMargin: gauge.height / 4
    text: ""
    color: "black"
    font.pixelSize: 20
  }

  // 関数呼び出し用シグナル
  signal setBackgroundColor(string text)
  signal setMaximumValue(double value)
  signal setMinimumValue(double value)
  signal setStepSize(double value)
  signal setValue(double value)
  signal setTopText(string text)
  signal setBottomText(string text)

  Component.onCompleted: {
    setBackgroundColor.connect(onSetBackgroundColor)
    setMaximumValue.connect(onSetMaximumValue);
    setMinimumValue.connect(onSetMinimumValue);
    setStepSize.connect(onSetStepSize);
    setValue.connect(onSetValue);
    setTopText.connect(onSetTopText);
    setBottomText.connect(onSetBottomText);
  }

  function onSetBackgroundColor(text)
  {
    rectangle.color = text;
  }

  function onSetMaximumValue(value)
  {
    gauge.maximumValue = value;
  }

  function onSetMinimumValue(value)
  {
    gauge.minimumValue = value;
  }

  function onSetStepSize(value)
  {
    gauge.stepSize = value;
  }

  function onSetValue(value)
  {
    gauge.value = value;
  }

  function onSetTopText(text)
  {
    topText.text = text;
  }

  function onSetBottomText(text)
  {
    bottomText.text = text;
  }
}
