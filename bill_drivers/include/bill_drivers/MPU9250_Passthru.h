/* 
   MPU9250_Passthru.h: class declaration for pass-through-mode MPU9250

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

#include "MPU9250.h"

class MPU9250_Passthru : public MPU9250 {

    public:

        MPU9250_Passthru(Ascale_t ascale, Gscale_t gscale, Mscale_t mscale, Mmode_t mmode, uint8_t sampleRateDivisor=0);

        Error_t begin(uint8_t i2cbus=1);

        bool checkNewAccelGyroData(void);

        bool checkNewMagData(void);

    protected:

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;

        virtual void readMPURegisters(uint8_t subAddress, uint8_t count, uint8_t * dest) override;

        virtual void writeRegister(uint8_t address, uint8_t subAddress, uint8_t data) override;

        virtual void readRegisters(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) override;

        uint8_t _mag;

    private:

        // Cross-platform support
        uint8_t  _i2c;

        virtual void writeAK8963Register(uint8_t subAddress, uint8_t data) override;

        virtual void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) override;

}; // class MPU9250_Passthru
