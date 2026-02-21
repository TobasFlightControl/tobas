import QtQuick 2.15
import QtQuick.Extras 1.4
import QtQuick.Controls.Styles 1.4

Rectangle {
  id: rectangle

  // 関数呼び出し用シグナル
  signal setBackgroundColor(string text)
  signal setBottomText(string text)
  signal setMaximumValue(double value)
  signal setMinimumValue(double value)
  signal setStepSize(double value)
  signal setTopText(string text)
  signal setValue(double value)

  function onSetBackgroundColor(text) {
    rectangle.color = text;
  }
  function onSetBottomText(text) {
    bottomText.text = text;
  }
  function onSetMaximumValue(value) {
    gauge.maximumValue = value;
  }
  function onSetMinimumValue(value) {
    gauge.minimumValue = value;
  }
  function onSetStepSize(value) {
    gauge.stepSize = value;
  }
  function onSetTopText(text) {
    topText.text = text;
  }
  function onSetValue(value) {
    gauge.value = value;
  }

  objectName: "rectangle"

  Component.onCompleted: {
    setBackgroundColor.connect(onSetBackgroundColor);
    setMaximumValue.connect(onSetMaximumValue);
    setMinimumValue.connect(onSetMinimumValue);
    setStepSize.connect(onSetStepSize);
    setValue.connect(onSetValue);
    setTopText.connect(onSetTopText);
    setBottomText.connect(onSetBottomText);
  }

  // CircularGauge: https://doc.qt.io/qt-5/qml-qtquick-extras-circulargauge.html
  CircularGauge {
    id: gauge
    anchors.fill: parent // 親ウィジェットの大きさに自動で合わせる
    objectName: "gauge"

    // CircularGaugeStyle: https://doc.qt.io/qt-5/qml-qtquick-controls-styles-circulargaugestyle.html
    // 親のメンバが更新されたらスタイルのメンバも自動で更新される
    style: CircularGaugeStyle {
      function computeTickmarkStepSize() {
        var step = gauge.maximumValue / 10;
        var units = [1000, 500, 200, 100, 50, 20, 10, 5, 2, 1];
        for (var i = 0; i < units.length; i++) {
          if (step >= units[i]) {
            return Math.ceil(step / units[i]) * units[i];
          }
        }
      }

      tickmarkStepSize: computeTickmarkStepSize()
    }
  }
  Text {
    id: topText
    anchors.horizontalCenter: gauge.horizontalCenter
    anchors.top: gauge.top
    anchors.topMargin: gauge.height / 4
    color: "black"
    font.pixelSize: 20
    objectName: "topText"
    text: ""
  }
  Text {
    id: bottomText
    anchors.bottom: gauge.bottom
    anchors.bottomMargin: gauge.height / 4
    anchors.horizontalCenter: gauge.horizontalCenter
    color: "black"
    font.pixelSize: 20
    objectName: "bottomText"
    text: ""
  }
}
