//
// Created by Robert Wang on 2022/8/9.
//

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace lsm_params {
    using matrix::SquareMatrix3f;
    // Linear State Model
    // y(t)   = C * x(t) + D * u(t)
    // x(t+1) = A * x(t) + B * u(t)

    // Model Parameters for Position Linear State Model
    static const SquareMatrix3f POS_MODEL_A((float[9]) {
            0.919489347204180,	0.0119464282604650,	-0.192819941473865,
            0.0528842239238510,	0.801093546380488,	-0.0477911531019410,
            0.162062006998199,	0.0650180881866930,	0.935833140249484
    });

    static const SquareMatrix3f POS_MODEL_B((float[9]) {
            -0.0691758229071862,	0.00595074207237158,	-0.191188917325680,
            0.0462025384767783,	-0.0272814074007897,	0.0585566178880740,
            0.129912405403274,	-0.0196449464552788,	0.0802167504129897
    });

    static const SquareMatrix3f POS_MODEL_C((float[9]) {
            -1.29290475615475,	-2.28817203216728,	0.892586174268364,
            -0.213618208134952,	-9.86203077194641,	-3.23728070561305,
            -0.00743160727436257,	0.483715174473085,	-1.38159159167620
    });

    static const SquareMatrix3f POS_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    // Model Parameters for Velocity Linear State Model
    static const SquareMatrix3f VEL_MODEL_A((float[9]) {
            0.945939218994339,	0.0155415345598237,	-0.00562246614333835,
            -0.00910969859791014,	0.947979166036600,	-0.00191799511674801,
            -0.00338321047831186,	-0.00347933085516419,	0.982333231305596
    });

    static const SquareMatrix3f VEL_MODEL_B((float[9]) {
            -0.0123846422546555,	-0.0183372869321554,	0.000120731875197880,
            0.0195965838189640,	-0.00924440720628244,	0.00158306512253179,
            0.00119068997988281,	-0.00202572095889213,	0.00384783178072417
    });

    static const SquareMatrix3f VEL_MODEL_C((float[9]) {
            -0.712991067636498,	2.32895129565492,	-0.303147868794428,
            -2.43902066914752,	-0.685470334851028,	-0.215106185430006,
            -0.214685774172540,	-0.228576242355074,4.64908878232118
    });

    static const SquareMatrix3f VEL_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    // Model Parameters for Attitude Linear State Model
    static const SquareMatrix3f ATT_MODEL_A((float[9]) {
            0.981221780404930,	-0.00109427049235634,	-0.00359494665471823,
            0.0374914672638471,	0.901624486168204,	0.0419470825095875,
            0.0462160873113424,	-0.0550882530642634,	0.995312131116061
    });

    static const SquareMatrix3f ATT_MODEL_B((float[9]) {
            -0.00789605958410498,	0.0462592907244172,	-0.00537797895856994,
            0.184654373552757,	-0.0880537453639859,	-0.112482194583249,
            0.0845364452056223,	-0.00231948994077510,	-0.101893826667246
    });

    static const SquareMatrix3f ATT_MODEL_C((float[9]) {
            0.766937499921799,	0.504910440421393,	-0.604406160954656,
            0.700953088781031,	0.117394890765249,	-0.115045355102882,
            1.23973456366793,	-0.0544777012060007,	-0.627309718687385
    });

    static const SquareMatrix3f ATT_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });
    
    // Model Parameters for Rate Linear State Model
    static const SquareMatrix3f RATE_MODEL_A((float[9]) {
            0.903264997045629,	-0.00859950576982774,	8.93798307789612e-05,
            0.00278084740044130,	0.898471214184887,	0.0219147976111119,
            0.0390515436868904,	-0.0217427961835949,	0.986958524976853
    });

    static const SquareMatrix3f RATE_MODEL_B((float[9]) {
            0.00992870424044070,	0.0148192549278530,	0.00367474607563924,
            -0.0158703636893475,	0.0152222254319134,	-0.00510224932791170,
            -0.00969964949930196,	-0.00251903289028068,	-0.00199267555580131
    });

    static const SquareMatrix3f RATE_MODEL_C((float[9]) {
            1.50603568650678,	-2.05925323061411,	-4.63734618012334,
            3.96969160288635,	3.20061888163443,	-0.988989417654861,
            7.09092718437484,	-4.47847818405435,	15.0007092988635
    });

    static const SquareMatrix3f RATE_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

}