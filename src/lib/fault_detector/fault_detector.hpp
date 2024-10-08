//
// Created by Robert Wang on 2022/4/25.
//

#pragma once

#include "common.hpp"
#include "Detector.hpp"
#include "CuSumVector.hpp"
#include "EMAVector.hpp"
#include "EMACuSumVector.hpp"
#include "HorzVertDetector.hpp"
#include "AbsErrorTimeWindowVector.hpp"
#include "SquareErrorTimeWindowVector.hpp"


namespace fault_detector {
    // Modify the usage to specify the detector class
    using GPSPosValidator = fault_detector::EMACuSumVector3f;
    using GPSVelValidator = fault_detector::EMACuSumVector3f;
    using MagValidator = fault_detector::EMACuSumVector3f;
    using BaroValidator = fault_detector::EMACuSumf;
    using GyroValidator = fault_detector::EMACuSumVector3f;
    using AccelValidator = fault_detector::EMACuSumVector3f;
}