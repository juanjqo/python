#include "../dqrobotics_module.h"

void init_DQ_TaskSpacePseudoInverseController_py(py::module& m)
{
    /*****************************************************
     *  DQ TaskSpacePseudoInverseController
     * **************************************************/
    py::class_<DQ_TaskSpacePseudoInverseController, DQ_KinematicController> dqtaskspacepseudoinversecontroller_py(m,"DQ_TaskSpacePseudoInverseController");
    dqtaskspacepseudoinversecontroller_py.def(py::init<DQ_Kinematics*>());
    dqtaskspacepseudoinversecontroller_py.def("compute_setpoint_control_signal",&DQ_TaskSpacePseudoInverseController::compute_setpoint_control_signal,"Computes the setpoint control signal.");
    dqtaskspacepseudoinversecontroller_py.def("compute_tracking_control_signal",&DQ_TaskSpacePseudoInverseController::compute_tracking_control_signal,"Computes the tracking control signal.");
}