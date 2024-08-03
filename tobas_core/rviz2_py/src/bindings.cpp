#include <pybind11/pybind11.h>

#include "../include/rviz2_py/visualization_frame_py.hpp"

using namespace rviz_common;
namespace py = pybind11;

PYBIND11_MODULE(librviz2, m)
{
  py::class_<VisualizationFramePy>(m, "VisualizationFrame")
    .def(py::init<QWidget*>())
    .def("setHelpPath", &VisualizationFramePy::setHelpPath)
    .def("setSplashPath", &VisualizationFramePy::setSplashPath)
    .def("initialize", &VisualizationFramePy::initialize)
    .def("setDisplayTitleFormat", &VisualizationFramePy::setDisplayTitleFormat)
    .def("getManager", &VisualizationFramePy::getManager)
    .def("loadDisplayConfig", &VisualizationFramePy::loadDisplayConfig)
    .def("saveDisplayConfig", &VisualizationFramePy::saveDisplayConfig)
    .def("getErrorMessage", &VisualizationFramePy::getErrorMessage)
    .def("setHideButtonVisibility", &VisualizationFramePy::setHideButtonVisibility);
}
