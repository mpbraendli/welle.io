/*
 *    Copyright (C) 2025
 *    Matthias P. Braendli (matthias.braendli@mpb.li)
 *
 *    This file is based on SDR-J
 *    Copyright (C) 2010, 2011, 2012
 *    Jan van Katwijk (J.vanKatwijk@gmail.com)
 *
 *    This file is part of the welle.io.
 *    Many of the ideas as implemented in welle.io are derived from
 *    other work, made available through the GNU general Public License.
 *    All copyrights of the original authors are recognized.
 *
 *    welle.io is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    welle.io is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with welle.io; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
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
    // EN 300 401 5.3.2.0 Packet mode - network level
    //
    // FIG 0/3 DG flag defines if data groups are used
    // DG=0 data groups are used
    // DG=1 data groups are not used
    //
    // FIG 0/14 signals the FEC Scheme:
    // scheme 00: no FEC (legacy)
    // scheme 01: FEC according to 5.3.5 FEC for MSC packet mode
    // (The FEC protects all packets in the sub-channel irrespective of their packet address)

    // This assumes we have data groups

    // packet header
    const uint8_t  length_indicator = getBits_2(data.data(), 0);
    const uint8_t  continuity_ix    = getBits_2(data.data(), 2);
    const bool     first            = getBits_1(data.data(), 4);
    const bool     last             = getBits_1(data.data(), 5);
    const uint16_t address          = getBits(data.data(), 6, 10);
    //const bool     command          = getBits_1(data.data(), 17);
    const uint8_t  data_length      = getBits_7(data.data(), 18);

    // includes the 3 bytes header length
    const uint8_t packet_length =
        (length_indicator == 0b00) ? 24 :
        (length_indicator == 0b01) ? 48 :
        (length_indicator == 0b10) ? 72 : 96;

    if (data.size() < packet_length * 8) {
        std::clog << "Packet " << data.size()/8 << ": " << (int)continuity_ix <<
            (first ? " F " : " f ") << (last ? " L " : " l ") <<
            " addr=" << (int)address <<
            " packetlen=" << (int)packet_length <<
            " datalen=" << (int)data_length <<
            std::endl;
        return;
    }

    if (packet_length == 24 && address == 1022) {
        /*
        uint8_t counter = getBits_4(data.data(), 2);
        std::clog << "Packet " << data.size()/8 << ": FEC " <<
            " counter=" << (int)counter <<
            std::endl;
        */
    }
    else {
        const bool crcvalid = check_CRC_bits(data.data(), packet_length*8);
        if (!crcvalid) {
            std::clog << "Packet " << data.size()/8 << ": " << (int)continuity_ix <<
                (first ? " F " : " f ") << (last ? " L " : " l ") <<
                " addr=" << (int)address <<
                " CRC invalid" <<
                std::endl;
        }
    }

    auto pkt = std::make_shared<packetmode::Packet>(data);
    auto packets = myRS.input_and_decode(pkt);

    // Valid packets need to be handed to a MSC DG decoder, e.g. mot-manager

    for (auto packet : packets) {
        const auto msc_data_group_header = data.data() + 3*8;
        const bool     extension_flag   = getBits_1(msc_data_group_header, 0);
        const bool     crc_flag         = getBits_1(msc_data_group_header, 1);
        const bool     segment_flag     = getBits_1(msc_data_group_header, 2);
        const bool     user_access_flag = getBits_1(msc_data_group_header, 3);
        const uint8_t  data_group_type  = getBits_4(msc_data_group_header, 4);
        const uint8_t  dg_continuity_ix = getBits_4(msc_data_group_header, 8);
        const uint8_t  repetition_index = getBits_4(msc_data_group_header, 12);

        if (data_group_type != 0) {
            std::clog << "Packet " << data.size()/8 <<
                " addr=" << (int)address <<
                " MSC cont ix=" << (int)dg_continuity_ix <<
                " repet ix=" << (int)repetition_index <<
                " has DGtype=" << int(data_group_type) << std::endl;
            return;
        }

        if (extension_flag) {
            std::clog << "Packet " << data.size()/8 <<
                " addr=" << (int)address <<
                " MSC cont ix=" << (int)dg_continuity_ix <<
                " repet ix=" << (int)repetition_index <<
                " has extension" << std::endl;
            return;
        }

        if (!crc_flag) {
            std::clog << "Packet " << data.size()/8 <<
                " addr=" << (int)address <<
                " MSC cont ix=" << (int)dg_continuity_ix <<
                " repet ix=" << (int)repetition_index <<
                " has no msc crc" << std::endl;
            return;
        }

        std::clog << "Packet " << data.size()/8 <<
            " addr=" << (int)address <<
            " MSC cont ix=" << (int)dg_continuity_ix <<
            " repet ix=" << (int)repetition_index <<
            std::endl;

        std::stringstream ss;
        ss << std::hex;
        for (size_t i = 4; i < data.size(); i++) {
            if (i % 8 == 0) ss << " ";
            ss << (int)data[i];
        }
        std::clog << ss.str() << std::endl;
    }

#warning "Do packet parsing and hand over to phi"
    // TODO myPacketDataHandler.onMSCDataGroup(std::vector<uint8_t>&& mscdg) = 0;
}

DabPacketData::~DabPacketData() {}
