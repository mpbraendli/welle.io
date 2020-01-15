/*
 *    Copyright (C) 2019
 *    Matthias P. Braendli (matthias.braendli@mpb.li)
 *
 *    Copyright (C) 2013
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *    Lazy Chair Programming
 *
 *    This file is part of the SDR-J (JSDR).
 *    SDR-J is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    SDR-J is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with SDR-J; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#pragma once

#include "dab-virtual.h"
#include <memory>
#include <atomic>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <cstdio>
#include "ringbuffer.h"
#include "energy_dispersal.h"
#include "radio-controller.h"

class DabProcessor;
class Protection;

class SubchannelHandler : public DabVirtual
{
    public:
        SubchannelHandler(
                  int16_t fragmentSize,
                  int16_t bitRate,
                  ProtectionSettings protection);
        virtual ~SubchannelHandler(void);
        SubchannelHandler(const SubchannelHandler&) = delete;
        SubchannelHandler& operator=(const SubchannelHandler&) = delete;

        int32_t process(const softbit_t *v, int16_t cnt);

    protected:
        void start();
        virtual void addtoFrame(const std::vector<uint8_t>& data) = 0;

    private:
        void run();
        std::atomic<bool> running;
        int16_t fragmentSize;
        int16_t bitRate;
        std::vector<uint8_t> outV;
        std::vector<softbit_t> interleaveData[16];
        EnergyDispersal energyDispersal;

        std::condition_variable  mscDataAvailable;
        std::mutex               myMutex;
        std::thread              myThread;

        std::unique_ptr<Protection> protectionHandler;
        RingBuffer<softbit_t> mscBuffer;

        const std::string dumpFileName;
};

class DabAudio : public SubchannelHandler {
    public:
        DabAudio(AudioServiceComponentType dabModus,
                  int16_t fragmentSize,
                  int16_t bitRate,
                  ProtectionSettings protection,
                  ProgrammeHandlerInterface& phi,
                  const std::string& dumpFileName);
        virtual ~DabAudio();

    protected:
        virtual void addtoFrame(const std::vector<uint8_t>& data);

    private:
        std::unique_ptr<DabProcessor> myProcessor;
        ProgrammeHandlerInterface& myProgrammeHandler;
};

class DabPacketData : public SubchannelHandler {
    public:
        DabPacketData(DataServiceComponentType dsctype,
                  int16_t fragmentSize,
                  int16_t bitRate,
                  ProtectionSettings protection,
                  PacketDataHandlerInterface& pdhi,
                  const std::string& dumpFileName);
        virtual ~DabPacketData();

    protected:
        virtual void addtoFrame(const std::vector<uint8_t>& data);

    private:
        std::unique_ptr<DabProcessor> myProcessor;
        PacketDataHandlerInterface& myPacketDataHandler;
};
