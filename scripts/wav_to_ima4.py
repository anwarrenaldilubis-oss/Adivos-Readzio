#!/usr/bin/env python3
"""Convert 16 kHz mono PCM WAV files to headered IMA ADPCM assets."""

import argparse
import re
import struct
import wave
from pathlib import Path

STEP_TABLE = (
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31,
    34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130,
    143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449,
    494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411,
    1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327, 3660, 4026,
    4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767,
)
INDEX_TABLE = (-1, -1, -1, -1, 2, 4, 6, 8)


def encode_nibble(sample: int, predictor: int, index: int) -> tuple[int, int, int]:
    step = STEP_TABLE[index]
    difference = sample - predictor
    nibble = 0
    if difference < 0:
        nibble = 8
        difference = -difference

    delta = step >> 3
    if difference >= step:
        nibble |= 4
        difference -= step
        delta += step
    if difference >= step >> 1:
        nibble |= 2
        difference -= step >> 1
        delta += step >> 1
    if difference >= step >> 2:
        nibble |= 1
        delta += step >> 2

    predictor += -delta if nibble & 8 else delta
    predictor = max(-32768, min(32767, predictor))
    index += INDEX_TABLE[nibble & 7]
    index = max(0, min(88, index))
    return nibble, predictor, index


def convert(source: Path) -> Path:
    with wave.open(str(source), "rb") as wav:
        if (wav.getnchannels() != 1 or wav.getsampwidth() != 2
                or wav.getframerate() != 16000 or wav.getcomptype() != "NONE"):
            raise ValueError(f"{source}: expected PCM mono 16-bit 16000 Hz WAV")
        pcm_bytes = wav.readframes(wav.getnframes())

    if len(pcm_bytes) < 2 or len(pcm_bytes) % 2:
        raise ValueError(f"{source}: WAV has no complete PCM samples")

    samples = struct.unpack(f"<{len(pcm_bytes) // 2}h", pcm_bytes)
    predictor = samples[0]
    index = 0
    encoded = bytearray()
    pending_nibble = None

    for sample in samples[1:]:
        nibble, predictor, index = encode_nibble(sample, predictor, index)
        if pending_nibble is None:
            pending_nibble = nibble
        else:
            encoded.append(pending_nibble | (nibble << 4))
            pending_nibble = None
    if pending_nibble is not None:
        encoded.append(pending_nibble)

    output_stem = re.sub(
        r"_+", "_", re.sub(r"[^A-Za-z0-9_]+", "_", source.stem)
    ).strip("_")
    output = source.with_name(f"{output_stem}.ima4")
    output.write_bytes(struct.pack("<IhBB", len(samples), samples[0], 0, 0) + encoded)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path, help="Directory containing source WAVs")
    args = parser.parse_args()
    wav_files = sorted(args.directory.glob("*_clean.wav"))
    if not wav_files:
        parser.error(f"no *_clean.wav files found in {args.directory}")

    source_bytes = 0
    output_bytes = 0
    for source in wav_files:
        result = convert(source)
        source_bytes += source.stat().st_size
        output_bytes += result.stat().st_size
        print(f"{source.name}: {source.stat().st_size} -> {result.stat().st_size} bytes")

    print(f"Total: {source_bytes} -> {output_bytes} bytes "
          f"({100 * output_bytes / source_bytes:.1f}% of original)")


if __name__ == "__main__":
    main()
