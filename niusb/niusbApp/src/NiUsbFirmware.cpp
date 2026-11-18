#include "NiUsbFirmware.h"

#include <cstring>
#include <fstream>
#include <iterator>

namespace niusb {

const std::vector<FirmwareFile> &firmwareFiles()
{
    static const std::vector<FirmwareFile> f = {
        {"usb621x.cfg", "f890e2c578976adbc0e47a1636a0dc8e8d21b69e01a385da9f30199b7a35f8e1"},
        {"usb621x0.cfg", "da1a4fe26435cbc20a8814c5e57c8ae861f69eb45211277bfa4fa6b0fa9b047a"},
        {"usb621x1.cfg", "a1d002660ac1a1044b51eb95b68c34dafa8b58877906d9a2114f6d37c53e877d"},
        {"SlicFpga.cfg", "2fcb1fb78ae454cbb328199e6a9e0e07b8a96423511196f6fb9f6863a1ee0be6"},
        {"STC3Lite.cfg", "3b72c4056816f37de5aa6940b207a3b638bd65b0eec50a4543f0d7f9e3d77af1"},
    };
    return f;
}

/* SHA-256 (FIPS 180-4), small and dependency-free. */
std::string sha256Hex(const Bytes &data)
{
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    Bytes m = data;
    uint64_t bits = uint64_t(data.size()) * 8;
    m.push_back(0x80);
    while (m.size() % 64 != 56)
        m.push_back(0);
    for (int i = 7; i >= 0; i--)
        m.push_back(uint8_t(bits >> (8 * i)));
    auto rotr = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
    for (size_t off = 0; off < m.size(); off += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; i++)
            w[i] = be32(&m[off + 4 * i]);
        for (int i = 16; i < 64; i++) {
            uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; i++) {
            uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            uint32_t ch = (e & f) ^ (~e & g);
            uint32_t t1 = hh + S1 + ch + k[i] + w[i];
            uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
            uint32_t t2 = S0 + mj;
            hh = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }
    Bytes out;
    for (uint32_t x : h)
        for (int i = 3; i >= 0; i--)
            out.push_back(uint8_t(x >> (8 * i)));
    return toHex(out);
}

static Bytes readFile(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        throw Error("NI firmware file missing: " + path +
                    " (copy it from a lab PC's NI-DAQmx install, NI-DAQ\\fpgaFiles\\miousb\\; geecs-epics decision 23)");
    return Bytes(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

Firmware Firmware::load(const std::string &dir, bool checkSums)
{
    std::vector<Bytes> files;
    for (const FirmwareFile &ff : firmwareFiles()) {
        Bytes b = readFile(dir + "/" + ff.name);
        if (checkSums) {
            std::string got = sha256Hex(b);
            if (got != ff.sha256)
                throw Error(std::string("NI firmware file ") + ff.name + " in " + dir +
                            " is not the NI-DAQmx 20 file this driver was proven with (sha256 " + got +
                            ", expected " + ff.sha256 + ")");
        }
        files.push_back(std::move(b));
    }
    const size_t MAIN = 16384;
    if (files[1].size() <= MAIN || files[2].size() <= MAIN)
        throw Error("NI firmware: usb621x0.cfg and usb621x1.cfg must each hold the 16 KB main image and a bank");
    Firmware fw;
    fw.loader = files[0];
    fw.main16k.assign(files[1].begin(), files[1].begin() + MAIN);
    if (!std::equal(fw.main16k.begin(), fw.main16k.end(), files[2].begin()))
        throw Error("NI firmware: usb621x0.cfg and usb621x1.cfg disagree on the main image");
    fw.bank0.assign(files[1].begin() + MAIN, files[1].end());
    fw.bank1.assign(files[2].begin() + MAIN, files[2].end());
    fw.fpga0 = files[3];
    fw.fpga1 = files[4];
    return fw;
}

} // namespace niusb
