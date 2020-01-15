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
 */

#include <iostream>
#include <vector>
#include "dab-constants.h"
#include "subchannel-handler.h"
#include "decoder_adapter.h"
#include "eep-protection.h"
#include "uep-protection.h"
#include "profiling.h"

//  fragmentsize == Length * CUSize
SubchannelHandler::SubchannelHandler(
        int16_t fragmentSize,
        int16_t bitRate,
        ProtectionSettings protection) :
    DabVirtual(),
    mscBuffer(64 * 32768)
{
    this->fragmentSize     = fragmentSize;
    this->bitRate          = bitRate;

    outV.resize(bitRate * 24);
    for (int i = 0; i < 16; i ++) {
        interleaveData[i].resize(fragmentSize);
    }

    using std::make_unique;

    if (protection.shortForm) {
        protectionHandler = make_unique<UEPProtection>(bitRate, protection.uepLevel);
    }
    else {
        const bool profile_is_eep_a =
            protection.eepProfile == EEPProtectionProfile::EEP_A;
        protectionHandler = make_unique<EEPProtection>(
                bitRate, profile_is_eep_a, (int)protection.eepLevel);
    }
}

void SubchannelHandler::start()
{
    running = true;
    myThread = std::thread(&SubchannelHandler::run, this);
}

SubchannelHandler::~SubchannelHandler()
{
    running = false;

    if (myThread.joinable()) {
        mscDataAvailable.notify_all();
        myThread.join();
    }
}

int32_t SubchannelHandler::process(const softbit_t *v, int16_t cnt)
{
    int32_t fr;

    if (mscBuffer.GetRingBufferWriteAvailable () < cnt)
        fprintf (stderr, "dab-concurrent: buffer full\n");

    while ((fr = mscBuffer.GetRingBufferWriteAvailable ()) <= cnt) {
        if (!running)
            return 0;
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    mscBuffer.putDataIntoBuffer(v, cnt);
    mscDataAvailable.notify_all();
    return fr;
}

const int16_t interleaveMap[] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};

void SubchannelHandler::run()
{
    int16_t i;
    int16_t countforInterleaver = 0;
    int16_t interleaverIndex    = 0;
    std::vector<softbit_t> data(fragmentSize);
    std::vector<softbit_t> tempX(fragmentSize);

    while (running) {
        std::unique_lock<std::mutex> lock(myMutex);
        while (running && mscBuffer.GetRingBufferReadAvailable() <= fragmentSize) {
            mscDataAvailable.wait(lock);
        }
        if (!running)
            break;

        // mscBuffer is threadsafe to access, no need to keep the lock
        lock.unlock();

        PROFILE(DAGetMSCData);
        mscBuffer.getDataFromBuffer(data.data(), fragmentSize);

        PROFILE(DADeinterleave);
        for (i = 0; i < fragmentSize; i ++) {
            tempX[i] = interleaveData[(interleaverIndex +
                    interleaveMap[i & 017]) & 017][i];
            interleaveData[interleaverIndex][i] = data[i];
        }
        interleaverIndex = (interleaverIndex + 1) & 0x0F;

        //  only continue when de-interleaver is filled
        if (countforInterleaver <= 15) {
            countforInterleaver ++;
            continue;
        }

        PROFILE(DADeconvolve);
        protectionHandler->deconvolve(tempX.data(), fragmentSize, outV.data());

        PROFILE(DADispersal);
        // and the inline energy dispersal
        energyDispersal.dedisperse(outV);

        PROFILE(DADecode);
        addtoFrame(outV);
        PROFILE(DADone);
    }
}

DabAudio::DabAudio(AudioServiceComponentType dabModus,
                  int16_t fragmentSize,
                  int16_t bitRate,
                  ProtectionSettings protection,
                  ProgrammeHandlerInterface& phi,
                  const std::string& dumpFileName) :
    SubchannelHandler(fragmentSize, bitRate, protection),
    myProgrammeHandler(phi)
{
    myProcessor = std::make_unique<DecoderAdapter>(
            myProgrammeHandler, bitRate, dabModus, dumpFileName);

    start();
}

void DabAudio::addtoFrame(const std::vector<uint8_t>& data)
{
    myProcessor->addtoFrame(data.data());
}

DabAudio::~DabAudio() {}

DabPacketData::DabPacketData(
        DataServiceComponentType dsctype,
        int16_t fragmentSize,
        int16_t bitRate,
        ProtectionSettings protection,
        PacketDataHandlerInterface& phdi,
        const std::string& dumpFileName) :
    SubchannelHandler(fragmentSize, bitRate, protection),
    myPacketDataHandler(phdi)
{
    start();
}

void DabPacketData::addtoFrame(const std::vector<uint8_t>& data)
{
    std::clog << "Got " << data.size() << " bytes of data" << std::endl;

#warning "Do packet parsing and hand over to phi"
    // TODO myPacketDataHandler.onMSCDataGroup(std::vector<uint8_t>&& mscdg) = 0;
}

DabPacketData::~DabPacketData() {}
