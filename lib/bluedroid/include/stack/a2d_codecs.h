/******************************************************************************
 *
 *  Copyright (C) 2026 Skye Green
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#ifndef A2D_CODECS_H
#define A2D_CODECS_H
#include "stack/a2d_api.h"
#include <stdint.h>
#if (A2D_INCLUDED == TRUE)

#define A2D_CODEC_SBC 0x003F0000
#define A2D_CODEC_MPEG_AUDIO 0x003F0001
#define A2D_CODEC_AAC 0x003F0002
#define A2D_CODEC_MPEG_USAC 0x003F0003
#define A2D_CODEC_ATRAC 0x003F0004
#define A2D_CODEC_APTX 0x004F0001
#define A2D_CODEC_APTX_HD 0x00D70024
#define A2D_CODEC_LDAC 0x012D00AA
#define A2D_CODEC_OPUS 0x00E00001
#define A2D_CODEC_INVALID 0x003F00FF // Invalid value for tA2D_CODEC

/// Standardized codec identifiers.
///
/// The codec identifier is 32 bits:
///  - Bits 0-15: Vendor ID, 0x003F (Bluetooth SIG, Inc) for standard codecs
///  - Bits 16-31: Vendor Specific Codec ID, assigned ID for standard codecs
typedef uint32_t tA2D_CODEC;

tA2D_CODEC A2D_GetCodecType(const uint8_t *p_info);

#endif  ///A2D_INCLUDED == TRUE
#endif /* A2D_CODECS_H */
