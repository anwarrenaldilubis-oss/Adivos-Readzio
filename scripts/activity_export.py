#!/usr/bin/env python3
"""Receive and safely merge Readzio's non-destructive activity export."""

from __future__ import annotations

import argparse
import csv
import os
import secrets
import sys
import tempfile
import time
import zlib
from pathlib import Path
from typing import Protocol

CSV_COLUMNS = (
    "sequence",
    "timestamp_ms",
    "session_id",
    "activity",
    "target",
    "result",
    "attempt",
)
VALID_ACTIVITIES = {"vowel_choice", "cloud_search"}
VALID_VOWELS = set("AIUEO")
DEFAULT_OUTPUT = (
    Path.home() / "Documents" / "Readzio_Progress" / "activity_history.csv"
)


class SerialPort(Protocol):
    def read(self, size: int) -> bytes: ...

    def readline(self) -> bytes: ...

    def write(self, data: bytes) -> int: ...


def _parse_data_line(line: str, nonce: str) -> tuple[dict[str, str], bytes]:
    parts = line.rstrip("\r\n").split(",")
    if len(parts) != 9 or parts[0] != "READZIO_DATA" or parts[1] != nonce:
        raise ValueError("Invalid export data frame")

    row = dict(zip(CSV_COLUMNS, parts[2:]))
    sequence = int(row["sequence"])
    timestamp_ms = int(row["timestamp_ms"])
    attempt = int(row["attempt"])
    if sequence <= 0 or timestamp_ms < 0 or attempt <= 0:
        raise ValueError("Export row contains an invalid numeric field")
    if len(row["session_id"]) != 16:
        raise ValueError("Export row contains an invalid session ID")
    int(row["session_id"], 16)
    if row["activity"] not in VALID_ACTIVITIES:
        raise ValueError("Export row contains an unknown activity")
    if row["target"] not in VALID_VOWELS or row["result"] not in {"true", "false"}:
        raise ValueError("Export row contains an invalid target or result")
    return row, line.encode("ascii")


def receive_transfer(
    serial_port: SerialPort, nonce: str, timeout_seconds: float
) -> list[dict[str, str]]:
    deadline = time.monotonic() + timeout_seconds
    next_request = 0.0
    expected_count: int | None = None
    rows: list[dict[str, str]] = []
    sequences: set[int] = set()
    crc = 0
    partial_line = bytearray()

    while time.monotonic() < deadline:
        if expected_count is None and time.monotonic() >= next_request:
            serial_port.write(f"EXPORT,{nonce}\n".encode("ascii"))
            next_request = time.monotonic() + 1.0

        chunk = serial_port.read(256)
        if not chunk:
            continue
        partial_line.extend(chunk)
        while b"\n" in partial_line:
            raw_line, _, remainder = partial_line.partition(b"\n")
            partial_line = bytearray(remainder)
            raw_line += b"\n"
            try:
                text = raw_line.decode("ascii")
            except UnicodeDecodeError:
                continue
            fields = text.rstrip("\r\n").split(",")

            if len(fields) >= 2 and fields[0] == "READZIO_ERROR" and fields[1] == nonce:
                detail = fields[2] if len(fields) > 2 else "unknown"
                raise RuntimeError(f"Device rejected export: {detail}")

            if expected_count is None:
                if (len(fields) == 4 and fields[0] == "READZIO_BEGIN"
                        and fields[1] == nonce):
                    if fields[2] != "1":
                        raise ValueError(f"Unsupported export version: {fields[2]}")
                    expected_count = int(fields[3])
                    if expected_count < 0:
                        raise ValueError("Device returned a negative record count")
                continue

            if len(fields) >= 2 and fields[0] == "READZIO_BEGIN" and fields[1] == nonce:
                continue
            if (len(fields) == 4 and fields[0] == "READZIO_END"
                    and fields[1] == nonce):
                end_count = int(fields[2])
                end_crc = int(fields[3], 16)
                if end_count != expected_count or len(rows) != expected_count:
                    raise ValueError("Export ended with an incomplete record count")
                if end_crc != crc:
                    raise ValueError("Export CRC mismatch; CSV was not changed")
                return rows
            if len(fields) == 9 and fields[0] == "READZIO_DATA" and fields[1] == nonce:
                row, wire_bytes = _parse_data_line(text, nonce)
                sequence = int(row["sequence"])
                if sequence in sequences:
                    raise ValueError(f"Duplicate sequence in transfer: {sequence}")
                sequences.add(sequence)
                rows.append(row)
                crc = zlib.crc32(wire_bytes, crc)

    raise TimeoutError("Timed out waiting for a complete, verified export")


def load_existing(path: Path) -> dict[int, dict[str, str]]:
    if not path.exists():
        return {}

    records: dict[int, dict[str, str]] = {}
    with path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if tuple(reader.fieldnames or ()) != CSV_COLUMNS:
            raise ValueError(f"{path} is not a Readzio activity CSV")
        for row in reader:
            if None in row or any(row.get(column) is None for column in CSV_COLUMNS):
                raise ValueError(f"{path} contains a malformed CSV row")
            sequence = int(row["sequence"])
            normalized = {column: row[column] for column in CSV_COLUMNS}
            if sequence in records and records[sequence] != normalized:
                raise ValueError(f"{path} contains conflicting sequence {sequence}")
            records[sequence] = normalized
    return records


def save_merged(path: Path, new_rows: list[dict[str, str]]) -> int:
    records = load_existing(path)
    prior_count = len(records)
    for row in new_rows:
        sequence = int(row["sequence"])
        if sequence in records and records[sequence] != row:
            raise ValueError(f"Conflicting record for sequence {sequence}")
        records[sequence] = row

    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            newline="",
            encoding="utf-8-sig",
            dir=path.parent,
            prefix=f"{path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary:
            temp_path = Path(temporary.name)
            writer = csv.DictWriter(temporary, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for sequence in sorted(records):
                writer.writerow(records[sequence])
            temporary.flush()
            os.fsync(temporary.fileno())
        os.replace(temp_path, path)
    except OSError:
        if temp_path is not None:
            try:
                temp_path.unlink()
            except OSError:
                pass
        raise

    return len(records) - prior_count


def append_rows(
    path: Path,
    rows: list[dict[str, str]],
    known_records: dict[int, dict[str, str]],
) -> int:
    pending: list[dict[str, str]] = []
    new_records: dict[int, dict[str, str]] = {}
    for row in rows:
        sequence = int(row["sequence"])
        existing = known_records.get(sequence, new_records.get(sequence))
        if existing is not None:
            if existing != row:
                raise ValueError(f"Conflicting record for sequence {sequence}")
            continue
        new_records[sequence] = row
        pending.append(row)

    if not pending:
        return 0

    path.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
        if needs_header:
            writer.writeheader()
        for row in pending:
            writer.writerow(row)
        csv_file.flush()
        os.fsync(csv_file.fileno())
    known_records.update(new_records)
    return len(pending)


def parse_live_line(line: str) -> dict[str, str]:
    fields = line.rstrip("\r\n").split(",")
    if len(fields) != 9 or fields[0] != "READZIO_LIVE":
        raise ValueError("Invalid live activity frame")
    payload = ",".join(fields[:-1]).encode("ascii")
    expected_crc = int(fields[-1], 16)
    if zlib.crc32(payload) != expected_crc:
        raise ValueError("Live activity CRC mismatch")
    row, _ = _parse_data_line(
        f"READZIO_DATA,0000000000000000,{','.join(fields[1:-1])}\n",
        "0000000000000000",
    )
    return row


def run_live_receiver(
    connection: SerialPort,
    serial_error: type[Exception],
    output: Path,
    sync_timeout: float,
) -> None:
    records = load_existing(output)
    sync_nonce = secrets.token_hex(8)
    connection.write(f"SYNC,{sync_nonce}\n".encode("ascii"))
    sync_deadline = time.monotonic() + sync_timeout
    sync_count: int | None = None
    sync_rows: list[dict[str, str]] = []
    sync_sequences: set[int] = set()
    sync_crc = 0
    print(f"Listening on {connection.port}; saving to {output}")
    print("Press Ctrl+C to stop. Device history is retained in flash.")

    while True:
        try:
            raw_line = connection.readline()
        except serial_error:
            raise
        if not raw_line:
            if sync_count is None and time.monotonic() >= sync_deadline:
                raise TimeoutError("No history sync response from the device")
            continue
        if len(raw_line) > 512:
            print("Ignored oversized serial line.", file=sys.stderr)
            continue
        try:
            line = raw_line.decode("ascii")
        except UnicodeDecodeError:
            continue
        fields = line.rstrip("\r\n").split(",")

        if (sync_count is None and len(fields) == 4
                and fields[0] == "READZIO_BEGIN" and fields[1] == sync_nonce):
            if fields[2] != "1":
                raise ValueError(f"Unsupported export version: {fields[2]}")
            sync_count = int(fields[3])
            if sync_count < 0:
                raise ValueError("Device returned a negative record count")
            continue

        if len(fields) == 9 and fields[0] == "READZIO_LIVE":
            try:
                row = parse_live_line(line)
                added = append_rows(output, [row], records)
            except ValueError as error:
                print(f"Ignored live record: {error}", file=sys.stderr)
                continue
            if added:
                print(f"Saved {row['activity']} {row['target']} "
                      f"{row['result']} (attempt {row['attempt']})")
            continue

        if len(fields) == 9 and fields[0] == "READZIO_DATA":
            if sync_count is None or fields[1] != sync_nonce:
                continue
            try:
                row, wire_bytes = _parse_data_line(line, sync_nonce)
            except ValueError as error:
                raise ValueError(f"Invalid history record: {error}") from error
            sequence = int(row["sequence"])
            if sequence in sync_sequences:
                raise ValueError(f"Duplicate sequence in history: {sequence}")
            sync_sequences.add(sequence)
            sync_rows.append(row)
            sync_crc = zlib.crc32(wire_bytes, sync_crc)
            continue

        if (sync_count is not None and len(fields) == 4
                and fields[0] == "READZIO_END" and fields[1] == sync_nonce):
            end_count = int(fields[2])
            end_crc = int(fields[3], 16)
            if end_count != sync_count or len(sync_rows) != sync_count:
                raise ValueError("History sync ended with an incomplete record count")
            if end_crc != sync_crc:
                raise ValueError("History sync CRC mismatch")
            added = append_rows(output, sync_rows, records)
            print(f"History sync complete: {len(sync_rows)} records, "
                  f"{added} new rows added.")
            sync_count = None
            sync_rows.clear()
            sync_sequences.clear()
            sync_crc = 0
            continue

        if (sync_count is None and len(fields) >= 2
                and fields[0] == "READZIO_ERROR" and fields[1] == sync_nonce):
            detail = fields[2] if len(fields) > 2 else "unknown"
            raise RuntimeError(f"Device rejected history sync: {detail}")


def list_ports() -> int:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2
    ports = list(list_ports.comports())
    if not ports:
        print("No serial COM ports detected.")
        return 0
    for port in ports:
        print(f"{port.device}: {port.description}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3",
                        help="Windows UART COM port (default: COM3)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT,
                        help="CSV path (default: Documents\\Readzio_Progress\\activity_history.csv)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Seconds to wait for the initial history sync")
    parser.add_argument("--list-ports", action="store_true")
    args = parser.parse_args()

    if args.list_ports:
        return list_ports()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")

    try:
        import serial
    except ImportError:
        print("pyserial is required: python -m pip install pyserial", file=sys.stderr)
        return 2

    try:
        while True:
            connection = serial.Serial()
            connection.port = args.port
            connection.baudrate = 115200
            connection.timeout = 0.5
            connection.write_timeout = 2
            connection.dtr = False
            connection.rts = False
            connection.dsrdtr = False
            connection.rtscts = False
            try:
                connection.open()
                run_live_receiver(
                    connection, serial.SerialException, args.output, args.timeout
                )
            except serial.SerialException as error:
                print(f"Serial disconnected ({error}); reconnecting in 2 seconds.",
                      file=sys.stderr)
                time.sleep(2)
            except (OSError, RuntimeError, TimeoutError, ValueError) as error:
                print(f"Receiver stopped: {error}", file=sys.stderr)
                return 1
            finally:
                if connection.is_open:
                    connection.close()
    except KeyboardInterrupt:
        print("\nStopped; CSV and device history are retained.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
