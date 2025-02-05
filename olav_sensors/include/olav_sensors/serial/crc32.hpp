/*
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 +                            _     _     _     _                            +
 +                           / \   / \   / \   / \                           +
 +                          ( O ) ( L ) ( A ) ( V )                          +
 +                           \_/   \_/   \_/   \_/                           +
 +                                                                           +
 +                  OLAV: Off-Road Light Autonomous Vehicle                  +
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

MIT License

Copyright (c) 2024 Dario Sirangelo

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <cstdint>

struct CRC32_s {
    void GenerateTable(uint32_t (&table)[256]) {
        uint32_t polynomial = 0xEDB88320;
        for(uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for(size_t j = 0; j < 8; j++) {
                if(c & 1) {
                    c = polynomial ^ (c >> 1);
                } else {
                    c >>= 1;
                }
            }
            table[i] = c;
        }
    }

    uint32_t Update(uint32_t (&table)[256],
                    uint32_t initial,
                    const void* buf,
                    size_t len) {
        uint32_t c = initial ^ 0xFFFFFFFF;
        const uint8_t* u = static_cast<const uint8_t*>(buf);
        for(size_t i = 0; i < len; ++i) {
            c = table[(c ^ u[i]) & 0xFF] ^ (c >> 8);
        }
        return c ^ 0xFFFFFFFF;
    }
};

class CRC32 {
  public:
    CRC32() : initial(0) { crc32_s.GenerateTable(table); }

    void Update(const uint8_t* buf, size_t len) {
        initial = crc32_s.Update(table, initial, (const void*)buf, len);
    }

    uint32_t GetValue() const { return initial; }

  private:
    uint32_t table[256];

    CRC32_s crc32_s;

    uint32_t initial;
};