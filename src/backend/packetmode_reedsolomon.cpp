/*
 *    Copyright (C) 2025
 *    Matthias P. Braendli (matthias.braendli@mpb.li)
 *
 *    Copyright (C) 2025
 *    flohoff
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

#include <stdexcept>
#include <iostream>
#include <string.h>
#include "packetmode_reedsolomon.h"

extern "C" {
#include <fec.h>
}

namespace packetmode {

#ifdef RSDEBUG
static void dump_hex(const char *prefix, uint8_t *buf, int size, int cols) {
    int     i;
    unsigned char   ch;
    char        sascii[cols+1];
    char        linebuffer[cols*4+1];

    sascii[cols]=0x0;

    for(i=0;i<size;i++) {
        ch=buf[i];
        if (i%cols == 0) {
            sprintf(linebuffer, "%04x ", i);
        }
        sprintf(&linebuffer[(i%cols)*3], "%02x ", ch);
        if (ch >= ' ' && ch <= '}')
            sascii[i%cols]=ch;
        else
            sascii[i%cols]='.';

        if (i%cols == (cols-1))
            printf("%s %s  %s\n", prefix, linebuffer, sascii);
    }

    /* i++ after loop */
    if (i%cols != 0) {
        for(;i%cols != 0;i++) {
            sprintf(&linebuffer[(i%cols)*3], "   ");
            sascii[i%cols]=' ';
        }

        printf("%s %s  %s\n", prefix, linebuffer, sascii);
    }
}
#endif

Packetmode_ReedSolomon::Packetmode_ReedSolomon()
{
    fecbuffer.resize(feccolumns*rows);
    buffer.resize(rows*columns);
    processbuffer.resize(rows*(columns+feccolumns));

    /* Symbol size 8 bit
     * Poly 0x11d
     * 16 bytes per row RS / FEC bytes
     * 0 padding (We do it before we decode)
     */
    rs_handle = init_rs_char(8, 0x11d, 0, 1, 16, 0);

    if (!rs_handle)
        throw std::runtime_error("RSDecoder: error while init_rs_char");
}

Packetmode_ReedSolomon::~Packetmode_ReedSolomon()
{
    if (rs_handle) {
        free_rs_char(rs_handle);
        rs_handle = nullptr;
    }
}

std::list<std::shared_ptr<Packet>> Packetmode_ReedSolomon::pkt_list()
{
    return pkts;
}

bool Packetmode_ReedSolomon::pkts_process_fec()
{
    uint8_t rstable[rows][columns+feccolumns];
    size_t  dptr=0;     /* Data ptr */
    size_t  fecpkts=0;
    size_t  pktbytes=0;

#define FEC_PKT_HDR_LENGTH  2
#define FEC_PKT_BYTES  22
    auto fec_pkt_bytes = [](int fec_count) {
        // EN 300 401 5.3.5.2 "there remain 6 unused bytes" in the 9th FEC packet
        return (fec_count < 8) ? FEC_PKT_BYTES : FEC_PKT_BYTES-6;
    };

    memset(rstable, 0, rows*(columns+feccolumns));

    for (const auto &pkt : pkts) {
        const auto& pbuf = pkt->data();

        if (pkt->is_fec()) {
            /*
             * FEC packets (should be 9 in our buffer)
             * Must be interleaved into columns from column 239 on
             */
            const int poff = pkt->fec_count() * FEC_PKT_BYTES;
            for (int i=0; i < fec_pkt_bytes(pkt->fec_count()); i++) {
                /* fprintf(stderr, "rstable[%d][%d = %d + (%d + %d) / %d]\n",
                    (poff+i) % rows,
                    columns + (poff+i) / rows,
                    columns, poff,i, rows); */
                rstable[(poff+i) % rows][columns + (poff+i) / rows] =
                    pbuf[FEC_PKT_HDR_LENGTH+i];
            }

            fecpkts++;
        }
        else {
            /* Overflowing buffer? */
            if (dptr + pkt->size() > rows*columns) {
                std::cerr << "overflow?\n";
                pkts.clear();
                return false;
            }

            /* Data packet - interleave into columns */
            for(size_t i=0;i<pkt->size();i++) {
                rstable[dptr % rows][pad + dptr / rows] = pbuf[i];
                dptr++;
            }
            pktbytes += pkt->size();
        }
    }

    /*
    std::cerr << "FEC pkts " << pkts.size()
        << " bytes " << pktbytes
        << " FEC packets " << fecpkts << std::endl;
        */

    if (pktbytes != TOTAL_APPLICATION_BYTES) {
        std::cerr << "Unable to run FEC - did not receive all packets" << std::endl;
        return false;
    }

    for (unsigned int r=0;r<rows;r++) {
        for (int& c : corr_pos)
            c = 0;

        int corr_count = decode_rs_char(rs_handle, rstable[r], corr_pos, 0);
        /* We need to copy back to packet buffers in case of corrected bytes.
         * As we copied them interleaved into the rows we need to walk through
         * again and if it matches to the corrected position copy back the byte.
         */

        if (corr_count < 0) {
            std::cerr << "Uncorrectable errors in FEC" << std::endl;
            return false;
        }

        // FIXME - Mark packets which may contain uncorrectable errors
        for(int i=0; i<corr_count; i++) {
            dptr = 0;
            unsigned int cpos=corr_pos[i];

            for (auto &pkt : pkts) {
                uint8_t *pbuf = pkt->data().data();

                /* Data packet - interleave into columns */
                for(size_t j=0;j<pkt->size();j++) {
                    if ((dptr % rows == r) && ((pad + dptr / rows) == cpos)) {
                        pbuf[j] = rstable[dptr % rows][pad + dptr / rows];
                    }
                    dptr++;
                }
            }
        }
    }

    return true;
}

std::list<std::shared_ptr<Packet>> Packetmode_ReedSolomon::input_and_decode(
        std::shared_ptr<Packet> pkt)
{
    pkts.push_back(pkt);

    /*
    auto calc_num_fec_packets = [&](){
        size_t num = 0;
        for (const auto& p : pkts) {
            if (p->is_fec())
                num += 1;
        }
        return num;
    }; */

    auto calc_app_size = [&](){
        size_t total_size = 0;
        for (const auto& p : pkts) {
            if (! p->is_fec())
                total_size += p->size();
        }
        return total_size;
    };

    while (calc_app_size() > TOTAL_APPLICATION_BYTES) {
        pkts.pop_front();
    }

    std::list<std::shared_ptr<Packet>> out;

    /* Trigger RS decoding once we get the last of the 9 FEC frames (0-8) */
    if (pkt->is_fec() && pkt->fec_count() == 8) {
        const auto app_size = calc_app_size();
        /*
        std::cerr << "Got last fec packet of " << calc_num_fec_packets() <<
            " app size " << calc_app_size() << "\n";
            */
        if (app_size == TOTAL_APPLICATION_BYTES) {
            if (pkts_process_fec()) {
                // pkts_process_fec will modify the packets in-place
                for (const auto& pkt : pkts) {
                    if (! pkt->is_fec()) {
                        out.push_back(pkt);
                    }
                }
            }
        }
        else {
            std::cerr << "Wrong size " << app_size << "\n";
        }

        pkts.clear();
    }

    /* Just a safety measure - possibly no FEC frames so we overflow memory */
    if (pkts.size() > 200) {
        std::cerr << "pkts overflow" << std::endl;
        pkts.clear();
    }

    return out;
}
}

#if PACKETMODE_TEST
// Test code to validate that the RS decoding works
#include <fstream>
#include <sstream>
#include <bitset>
#include "MathHelper.h"
int main(int argc, char **argv)
{
    std::vector<std::vector<uint8_t>> packets;
    std::ifstream file("packetmode_reedsolomon.dat");
    std::string line;

    while (std::getline(file, line)) {
        std::vector<uint8_t> row;
        std::istringstream iss(line);
        std::string binaryStr;

        // Split the line into 24 binary strings
        for (int i = 0; i < 24; ++i) {
            size_t pos = line.find(' ');
            binaryStr = line.substr(0, pos);
            line.erase(0, pos + 1);

            // Convert binary string to uint8_t
            uint8_t value = static_cast<uint8_t>(std::bitset<8>(binaryStr).to_ulong());
            row.push_back(value);
        }
        packets.push_back(row);
    }

    packetmode::Packetmode_ReedSolomon myRS;
    std::list<std::shared_ptr<packetmode::Packet>> corrected_packets;

    for (const auto& data : packets) {
        auto packet = std::make_shared<packetmode::Packet>();
        packet->load_bytes(data);
        fprintf(stderr, "HEX %d", packet->address());
        for (auto dat = packet->data().cbegin(); dat != packet->data().cend(); ++dat) {
            fprintf(stderr, " %08b", *dat);
        }
        fprintf(stderr, "\n");

        corrected_packets = myRS.input_and_decode(packet);
        if (corrected_packets.size())
            fprintf(stderr, "OUT = %zu\n", corrected_packets.size());
    }

    if (packets.size() == corrected_packets.size() + 9) {
        size_t i = 0;
        for (const auto& corrected_packet : corrected_packets) {

            const auto orig_packet = std::make_shared<packetmode::Packet>();
            orig_packet->load_bytes(packets.at(i++));

            if (*orig_packet != *corrected_packet) {
                fprintf(stderr, "%zu ORIG %d",i , orig_packet->address());
                for (auto dat : orig_packet->data()) {
                    fprintf(stderr, " %08b", dat);
                }
                fprintf(stderr, "\n");

                fprintf(stderr, "%zu CORR %d",i , corrected_packet->address());
                for (auto dat : corrected_packet->data()) {
                    fprintf(stderr, " %08b", dat);
                }
                fprintf(stderr, "\n");
            }
        }
    }
}

#endif
