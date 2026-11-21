import QtQuick 2.15

Rectangle {
  id: rectangle

  property int labelHeight: 20
  property int scaleWidth: 40
  property bool showScale: width >= scaleWidth + 12

  // Signal for function calls.
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

  TextMetrics {
    id: topMetrics
    font: topText.font
    text: topText.text
  }
  TextMetrics {
    id: bottomMetrics
    font: bottomText.font
    text: bottomText.text
  }
  Text {
    id: topText
    anchors.top: parent.top
    elide: Text.ElideMiddle
    font.pointSize: 11
    height: rectangle.labelHeight
    horizontalAlignment: Text.AlignHCenter
    leftPadding: Math.min(Math.max(0, width - topMetrics.advanceWidth), rectangle.showScale ? rectangle.scaleWidth : 0)
    objectName: "topText"
    verticalAlignment: Text.AlignVCenter
    width: Math.max(0, parent.width - 8)
    x: 4
  }
  Item {
    id: gauge

    property real fraction: maximumValue > minimumValue ? Math.max(0, Math.min(1, (value - minimumValue) / (maximumValue - minimumValue))) : 0
    property real maximumValue: 100
    property real minimumValue: 0
    property real stepSize: 1
    property real value: 0

    anchors.bottom: bottomText.visible ? bottomText.top : parent.bottom
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: topText.bottom
    objectName: "gauge"
    width: Math.min(parent.width, (rectangle.showScale ? rectangle.scaleWidth : 0) + 52)

    Rectangle {
      id: pipe
      anchors.bottom: parent.bottom
      anchors.bottomMargin: rectangle.showScale ? 8 : 0
      anchors.right: parent.right
      anchors.top: parent.top
      anchors.topMargin: rectangle.showScale ? 8 : 0
      border.color: "#808080"
      border.width: Math.min(2, Math.max(0, (width - 1) / 2))
      clip: true
      color: "white"
      width: Math.max(1, Math.min(48, parent.width - (rectangle.showScale ? rectangle.scaleWidth : 0)))

      Rectangle {
        anchors.bottom: parent.bottom
        anchors.left: parent.left
        anchors.margins: pipe.border.width
        anchors.right: parent.right
        color: "#00bcd4"
        height: Math.max(0, (pipe.height - 2 * pipe.border.width) * gauge.fraction)
      }
    }
    Repeater {
      model: rectangle.showScale ? 6 : 0

      Text {
        property real tickValue: gauge.minimumValue + (gauge.maximumValue - gauge.minimumValue) * index / 5

        font.pointSize: 8
        height: 16
        horizontalAlignment: Text.AlignRight
        text: Math.abs(tickValue) >= 1000 ? Number((tickValue / 1000).toPrecision(3)) + "k" : Number(tickValue.toPrecision(3)).toString()
        verticalAlignment: Text.AlignVCenter
        width: rectangle.scaleWidth - 4
        x: 0
        y: pipe.y + (pipe.height - height) * (1 - index / 5)
      }
    }
  }
  Text {
    id: bottomText
    anchors.bottom: parent.bottom
    elide: Text.ElideMiddle
    font.pointSize: 11
    height: rectangle.labelHeight
    horizontalAlignment: Text.AlignHCenter
    leftPadding: Math.min(Math.max(0, width - bottomMetrics.advanceWidth), rectangle.showScale ? rectangle.scaleWidth : 0)
    objectName: "bottomText"
    verticalAlignment: Text.AlignVCenter
    visible: rectangle.height >= 180
    width: parent.width
  }
}
