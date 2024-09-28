import QtQuick 2.15
import QtQuick.Extras 1.4

Rectangle {
  id: rectangle

  /* CircularGauge: https://doc.qt.io/qt-5/qml-qtquick-extras-circulargauge.html#stepSize-prop */
  CircularGauge {
    id: gauge
    maximumValue: 100
    minimumValue: 0
    stepSize: 1
    tickmarksVisible: true
    value: 0
    anchors.centerIn: parent
  }

  // 関数呼び出し用シグナル
  signal setMaximumValue(double max_value)
  signal setMinimumValue(double min_value)
  signal setStepSize(double step_size)
  signal setValue(double value)

  Component.onCompleted: {
    setMaximumValue.connect(onSetMaximumValue);
    setMinimumValue.connect(onSetMinimumValue);
    setStepSize.connect(onSetStepSize);
    setValue.connect(onSetValue);
  }

  function onSetMaximumValue(max_value)
  {
    gauge.maximumValue=max_value;
  }

  function onSetMinimumValue(min_value)
  {
    gauge.minimumValue=min_value;
  }

  function onSetStepSize(step_size)
  {
    gauge.stepSize=step_size;
  }

  function onSetValue(value)
  {
    gauge.value=value;
  }
}
