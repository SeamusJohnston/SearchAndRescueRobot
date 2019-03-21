  /* 
   MPU6x00.h: Superclass declaration for MPU6000 and MPU6500

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

#include "MPU6xx0.h"

class MPU6x00 : public MPU6xx0 {

    public:

        MPU6x00(Ascale_t ascale, Gscale_t gscale, uint8_t sampleRateDivisor=0);

    protected:

        Error_t begin(void);

        virtual void writeMPURegister(uint8_t subAddress, uint8_t data) override;
}; 
