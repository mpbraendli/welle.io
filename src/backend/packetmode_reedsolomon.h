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

#include <vector>
#include <cstdint>
#include <memory>
#include <list>

namespace packetmode {

class Packet {
    public:
        Packet(const std::vector<uint8_t> &bits)
        {
            const uint8_t *bitbuffer = bits.data();

            buffer.resize(bits.size() / 8);

            for (std::size_t i=0;i<bits.size()/8;i++) {
                uint8_t k=0;
                for (int j = 0; j < 8; j ++) {
                    k=k<<1|(bitbuffer[8 * i + j] & 01);
                }
                buffer[i]=k;
            }
        };

        std::size_t size() const {
            return buffer.size();
        }

        std::vector<uint8_t>& data() {
            return buffer;
        }

        int address() const {
            return (((buffer[0]) & 0x3) << 8 | (buffer[1]));
        }

        /* EN 300 401 - 5.3.5.2 - FEC for MSC packet Mod
         * Address: this 10-bit field shall take the binary value "1111111110" (1022).
         */
        bool is_fec() const {
            return address() == 1022;
        };

        /* EN 300 401 - 5.3.5.2 - Packet header Counter b13 .. b10
         */
        short fec_count() const {
            return ((buffer[0] >> 2) & 0xf);
        };

    private:
        std::vector<uint8_t> buffer;
};

class Packetmode_ReedSolomon {
    public:
        Packetmode_ReedSolomon();
        virtual ~Packetmode_ReedSolomon();

        std::list<std::shared_ptr<Packet>> input_and_decode(std::shared_ptr<Packet> pkt);
        std::list<std::shared_ptr<Packet>> pkt_list();

    private:
        std::vector<uint8_t> buffer;
        std::vector<uint8_t> processbuffer;

        int corr_pos[10];

        void *rs_handle;

        std::vector<uint8_t> fecbuffer;

        constexpr static size_t TOTAL_APPLICATION_BYTES = 2256;
        constexpr static size_t TOTAL_RS_BYTES = 192;
        constexpr static size_t TOTAL_FEC_FRAME_BYTES = TOTAL_APPLICATION_BYTES + TOTAL_RS_BYTES;
        // In total, there will be 2256 bytes of application data, and 192
        // bytes of RS data in the packets we receive.
        // Application packets can have lengths 24, 48, 72 or 96. FEC packets
        // have length 24.
        std::list<std::shared_ptr<Packet>> pkts;

        // See ETSI EN 300 401 Figure 17
        constexpr static unsigned int columns = 239;
        constexpr static unsigned int rows = 12;
        constexpr static unsigned int feccolumns = 16;
        constexpr static unsigned int framelength = 24;
        constexpr static unsigned int frames = 9;
        constexpr static unsigned int pad = 51;

        int pktvalid = 0;

        bool pkts_process_fec();
};
}
