/* Loads NI firmware from a folder with its checks on: prints "ok" or the
 * refusal (which names the file and both sums). */
#include <iostream>

#include "NiUsbFirmware.h"

int main(int argc, char **argv)
{
    if (argc != 2)
        return 2;
    try {
        niusb::Firmware::load(argv[1]);
        std::cout << "ok\n";
        return 0;
    } catch (const std::exception &e) {
        std::cout << e.what() << "\n";
        return 1;
    }
}
