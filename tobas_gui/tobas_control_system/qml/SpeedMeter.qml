import QtQuick

Rectangle {
  id: rectangle

  // Keep the interface used by SpeedmeterWidget while replacing Qt Quick Extras.
  signal setBackgroundColor(string text)
  signal setBottomText(string text)
  signal setMaximumValue(double value)
  signal setMinimumValue(double value)
  signal setStepSize(double value)
  signal setTopText(string text)
  signal setValue(double value)

  objectName: "rectangle"

  onSetBackgroundColor: text => rectangle.color = text
  onSetBottomText: text => bottomText.text = text
  onSetMaximumValue: value => gauge.maximumValue = value
  onSetMinimumValue: value => gauge.minimumValue = value
  onSetStepSize: value => gauge.stepSize = value
  onSetTopText: text => topText.text = text
  onSetValue: value => gauge.value = value

  Item {
    id: gauge

    property real maximumValue: 100
    property real minimumValue: 0
    property real stepSize: 0
    property real value: 0

    anchors.fill: parent
    objectName: "gauge"

    onMaximumValueChanged: dial.requestPaint()
    onMinimumValueChanged: dial.requestPaint()
    onStepSizeChanged: dial.requestPaint()
    onValueChanged: dial.requestPaint()

    Canvas {
      id: dial

      anchors.fill: parent

      onPaint: {
        const ctx = getContext("2d");
        ctx.reset();
        const radius = Math.min(width, height) * 0.46;
        if (radius <= 0 || gauge.maximumValue <= gauge.minimumValue)
          return;
        ctx.translate(width / 2, height / 2);
        ctx.strokeStyle = "#444444";
        ctx.fillStyle = "black";
        ctx.lineWidth = 2;
        ctx.font = Math.max(8, radius * 0.13) + "px sans-serif";
        ctx.textAlign = "center";
        ctx.textBaseline = "middle";

        const range = gauge.maximumValue - gauge.minimumValue;
        const rawStep = range / 10;
        const magnitude = Math.pow(10, Math.floor(Math.log10(rawStep)));
        const step = Math.ceil(rawStep / magnitude) * magnitude;
        for (let tick = gauge.minimumValue; tick <= gauge.maximumValue; tick += step) {
          const angle = (135 + 270 * (tick - gauge.minimumValue) / range) * Math.PI / 180;
          const x = Math.cos(angle);
          const y = Math.sin(angle);
          ctx.beginPath();
          ctx.moveTo(radius * 0.9 * x, radius * 0.9 * y);
          ctx.lineTo(radius * x, radius * y);
          ctx.stroke();
          ctx.fillText(Number(tick.toPrecision(6)).toString(), radius * 0.73 * x, radius * 0.73 * y);
        }

        const steppedValue = gauge.stepSize > 0 ? gauge.minimumValue + Math.round((gauge.value - gauge.minimumValue) / gauge.stepSize) * gauge.stepSize : gauge.value;
        const fraction = Math.max(0, Math.min(1, (steppedValue - gauge.minimumValue) / range));
        const angle = (135 + 270 * fraction) * Math.PI / 180;
        ctx.strokeStyle = "#c62828";
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(radius * 0.85 * Math.cos(angle), radius * 0.85 * Math.sin(angle));
        ctx.stroke();
        ctx.fillStyle = "#444444";
        ctx.beginPath();
        ctx.arc(0, 0, radius * 0.045, 0, 2 * Math.PI);
        ctx.fill();
      }
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
