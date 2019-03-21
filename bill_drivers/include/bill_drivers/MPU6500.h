  /* 
   MPU6500.h: Class declaration for MPU6500

   Copyright (C) 2018 Simon D. Levy

   This file is part of MPU.

   MPU is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   MPU is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with MPU.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include "MPU6x00.h"

class MPU6500 : public MPU6x00 {

    friend class MPU9250; // eventually should probably subclass MPU9250 from MPU6500

    public:

        MPU6500(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor=0);

        Error_t begin(void);

        bool checkNewData(void);

        void readGyrometer(float & gx, float & gy, float & gz);

    protected:

        // Register map
        static const uint8_t SELF_TEST_X_GYRO  = 0x00;                  
        static const uint8_t SELF_TEST_Y_GYRO  = 0x01;
        static const uint8_t SELF_TEST_Z_GYRO  = 0x02;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;
}; 
