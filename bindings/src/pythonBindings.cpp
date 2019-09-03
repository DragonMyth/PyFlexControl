//Written by Henry M. Clever. November 15, 2018.

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

#include "demo/main_python.cpp" //we can do this because of the FleX_INCLUDE_DIR in FindFleX.cmake
#include <stdio.h>

int add(int i, int j) {
	return 100; //deadzones[1];
}

namespace py = pybind11;

PYBIND11_MODULE(bindings, m) {
	m.doc() = "pybind11 passing plugin";


	m.def("update_frame", &UpdateFrame,"update");
	m.def("sdl_main", &SDLMain,
	 py::return_value_policy::automatic);
	m.def("initialize", &initialize, "float init");
	m.def("destroy_scene", &destroy_scene, py::return_value_policy::automatic);

	m.def("RandInit", &RandInit, "RandInit");
	m.def("chooseScene", &chooseScene, "chooseScene");
	m.def("get_state", &getState, "getState");
	m.def("setSceneRandSeed", &setSceneRandSeed, "setSceneRandSeed");
	m.def("resetScene", &Reset, "resetScene");
	m.def("getDt", &getDt, "getDt");
	m.def("setDt", &setDt, "setDt");
	m.def("getNumParticles", &getNumInstances, "getNumParticles");

	m.def("getNumInstances", &getNumInstances, "getNumInstances");
	m.def("getAllCenters",&getAllCenters,"getAllCenters");
	m.def("setVisualize",&setVisualization,"setVisualization");
#ifdef VERSION_INFO
	m.attr("__version__") = VERSION_INFO;
#else
	m.attr("__version__") = "dev";
#endif
}

