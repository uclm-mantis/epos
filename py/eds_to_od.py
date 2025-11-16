#!/usr/bin/env python3
import argparse
import configparser
import csv
import re
from pathlib import Path

# ---------- utilidades ----------

SECTION_RE = re.compile(r"^(?P<idx>[0-9A-Fa-f]{4})(?:sub(?P<sub>\d+))?$")

EDS_DATATYPES = {
    0x0001: "BOOLEAN",
    0x0002: "INTEGER8",
    0x0003: "INTEGER16",
    0x0004: "INTEGER32",
    0x0005: "UNSIGNED8",
    0x0006: "UNSIGNED16",
    0x0007: "UNSIGNED32",
    0x0008: "REAL32",
    0x0009: "STRING8",
    0x000A: "OCTET_STRING",
    0x000B: "UNICODE_STRING",
    0x000F: "DOMAIN",
    0x0011: "REAL64",
    0x001B: "UNSIGNED64",
}

ACCESS_MAP = {
    "ro": "RO",
    "rw": "RW",
    "wo": "WO",
    "rwr": "RW",
    "rww": "RW",
    "const": "CONST",
    "dec": "RO",
}

# rangos de objetos de comunicación que en tu od.csv van casi siempre a 0/0
COMM_RANGES = [
    (0x1000, 0x11FF),
    (0x1200, 0x13FF),
    (0x1400, 0x15FF),
    (0x1600, 0x17FF),
    (0x1800, 0x19FF),
    (0x1A00, 0x1BFF),
]

# si quieres excluir objetos “de info” de maxon como los 0x2003...
MAXON_KNOWN_INFO = {0x2003, 0x2004, 0x200A}


def in_ranges(value, ranges):
    return any(lo <= value <= hi for (lo, hi) in ranges)


def parse_eds(path: Path) -> configparser.ConfigParser:
    cfg = configparser.ConfigParser()
    cfg.optionxform = str  # no bajar a minúsculas las keys
    text = path.read_text(encoding="latin-1", errors="ignore")
    cfg.read_string(text)
    return cfg


def decode_datatype(raw: str) -> str:
    if not raw:
        return ""
    raw = raw.strip()
    try:
        v = int(raw, 0)
    except ValueError:
        return raw
    return EDS_DATATYPES.get(v, raw)


def decode_access(raw: str) -> str:
    if not raw:
        return ""
    return ACCESS_MAP.get(raw.lower(), raw.upper())


def build_index_has_subs(cfg: configparser.ConfigParser):
    index_has_subs = {}
    for sec in cfg.sections():
        m = SECTION_RE.match(sec)
        if not m:
            continue
        idx = m.group("idx").upper()
        sub = m.group("sub")
        index_has_subs.setdefault(idx, False)
        if sub is not None:
            index_has_subs[idx] = True
    return index_has_subs


def pdo_suffix(idx: int) -> str:
    """Devuelve el sufijo ' in RxPDO-<n>' / ' in TxPDO-<n>' según el rango."""
    if 0x1400 <= idx <= 0x15FF:  # RxPDO Communication Parameter
        n = idx - 0x1400 + 1
        return f" in RxPDO {n}"
    if 0x1600 <= idx <= 0x17FF:  # RxPDO Mapping Parameter
        n = idx - 0x1600 + 1
        return f" in RxPDO {n}"
    if 0x1800 <= idx <= 0x19FF:  # TxPDO Communication Parameter
        n = idx - 0x1800 + 1
        return f" in TxPDO {n}"
    if 0x1A00 <= idx <= 0x1BFF:  # TxPDO Mapping Parameter
        n = idx - 0x1A00 + 1
        return f" in TxPDO {n}"
    return ""


def eds_to_rows(cfg: configparser.ConfigParser):
    index_has_subs = build_index_has_subs(cfg)
    rows = []

    # detectar si es maxon para aplicar una pequeña lista negra
    vendor = cfg.get("DeviceInfo", "VendorName", fallback="").lower()
    is_maxon = "maxon" in vendor

    # contador para desambiguar descripciones duplicadas (p. ej. "Serial Number")
    desc_count = {}

    for sec_name in cfg.sections():
        m = SECTION_RE.match(sec_name)
        if not m:
            continue

        idx_hex = m.group("idx").upper()
        sub = m.group("sub")
        idx = int(idx_hex, 16)

        # si tiene subíndices y esta es la sección base -> no la exportamos
        if index_has_subs.get(idx_hex, False) and sub is None:
            continue

        sec = cfg[sec_name]

        # subíndice
        if sub is None:
            subidx = 0
            sub_str = "0x0"
        else:
            subidx = int(sub)
            sub_str = f"0x{subidx:X}".lower().replace("X", "x")

        desc = sec.get("ParameterName", "").strip()
        datatype = decode_datatype(sec.get("DataType", "").strip())
        access_raw = sec.get("AccessType", "").strip()
        access = decode_access(access_raw)

        pdomap = sec.get("PDOMapping", "").strip()
        objflags_raw = sec.get("ObjFlags", "").strip()

        # Añadir sufijo de contexto para objetos de PDO (comm + mapping)
        suf = pdo_suffix(idx)
        # Solo añadimos sufijo si la descripción no menciona ya RxPDO/TxPDO/PDO
        if suf and desc and "pdo" not in desc.lower():
            desc = f"{desc}{suf}"

        # por defecto
        rxpdo = 0
        txpdo = 0

        # 1) objetos de comunicación -> 0/0
        if in_ranges(idx, COMM_RANGES):
            rxpdo = 0
            txpdo = 0
        else:
            # 2) si el EDS dice que es mapeable...
            if pdomap == "1":
                ar = access_raw.lower()
                if "w" in ar:   # tiene alguna forma de write
                    rxpdo = 1
                if "r" in ar:   # tiene lectura
                    txpdo = 1

            # 3) si no, intentamos con ObjFlags (maxon lo pone bastante)
            elif objflags_raw:
                try:
                    flags = int(objflags_raw, 0)
                except ValueError:
                    flags = 0
                if flags & 0x1:
                    rxpdo = 1
                if flags & 0x2:
                    txpdo = 1

            # 4) ajustes específicos maxon: hay objetos de info que no quieres como PDO
            if is_maxon and idx in MAXON_KNOWN_INFO:
                rxpdo = 0
                txpdo = 0

        # Desambiguación de nombres duplicados (p. ej. "Serial Number", etc.)
        base_desc = desc or ""
        cnt = desc_count.get(base_desc, 0)
        if cnt >= 1 and base_desc:
            desc = f"{base_desc} ({cnt+1})"
        desc_count[base_desc] = cnt + 1

        rows.append({
            "index": f"0x{idx_hex}",
            "subindex": sub_str,
            "description": desc,
            "type": datatype,
            "access": access,
            "rxPDO": str(rxpdo),
            "txPDO": str(txpdo),
        })

    # ordenar por índice y subíndice
    rows.sort(key=lambda r: (int(r["index"], 16), int(r["subindex"], 16)))
    return rows


def write_csv(rows, out_path: Path):
    fieldnames = ["index", "subindex", "description", "type", "access", "rxPDO", "txPDO"]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main():
    ap = argparse.ArgumentParser(description="Genera od.csv a partir de una EDS de CANopen (maxon compatible)")
    ap.add_argument("eds", help="archivo .eds de entrada")
    ap.add_argument("-o", "--output", default="od.csv", help="CSV de salida")
    args = ap.parse_args()

    cfg = parse_eds(Path(args.eds))
    rows = eds_to_rows(cfg)
    write_csv(rows, Path(args.output))
    print(f"[+] Generado {args.output} con {len(rows)} objetos")


if __name__ == "__main__":
    main()
