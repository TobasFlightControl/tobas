from rviz import bindings as rviz


def create_rviz_frame(config_path: str):
    # Setup frame
    # cf. RViz Python Tutorial: https://docs.ros.org/en/indigo/api/rviz_python_tutorial/html/
    reader = rviz.YamlConfigReader()
    config = rviz.Config()
    reader.readFile(config, config_path)

    # Setup Visualization Frame
    # https://docs.ros.org/en/jade/api/rviz/html/c++/visualization__frame_8h_source.html
    frame = rviz.VisualizationFrame()
    frame.setSplashPath("")
    frame.initialize()
    frame.load(config)
    frame.setMenuBar(None)
    frame.setStatusBar(None)
    frame.setHideButtonVisibility(False)
    frame.setStyleSheet("QSizeGrip { width: 0px; height: 0px; }")  # Remove sizegrip

    return frame
