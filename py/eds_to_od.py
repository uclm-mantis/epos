#!/usr/bin/env python3
import argparse
import configparser
import csv
import re
from pathlib import Path

# ---------- utilidades de parseo EDS ----------

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
    0x0015: "INTEGER64",
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

COMM_RANGES = [
    (0x1000, 0x11FF),
    (0x1200, 0x13FF),
    (0x1400, 0x15FF),
    (0x1600, 0x17FF),
    (0x1800, 0x19FF),
    (0x1A00, 0x1BFF),
]

MAXON_KNOWN_INFO = {0x2003, 0x2004, 0x200A}


def in_ranges(value, ranges):
    return any(lo <= value <= hi for (lo, hi) in ranges)


def parse_eds(path: Path) -> configparser.ConfigParser:
    cfg = configparser.ConfigParser()
    cfg.optionxform = str
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
    if 0x1400 <= idx <= 0x15FF:
        return f" in RxPDO {idx - 0x1400 + 1}"
    if 0x1600 <= idx <= 0x17FF:
        return f" in RxPDO {idx - 0x1600 + 1}"
    if 0x1800 <= idx <= 0x19FF:
        return f" in TxPDO {idx - 0x1800 + 1}"
    if 0x1A00 <= idx <= 0x1BFF:
        return f" in TxPDO {idx - 0x1A00 + 1}"
    return ""


def eds_to_rows(cfg: configparser.ConfigParser):
    index_has_subs = build_index_has_subs(cfg)
    rows = []

    vendor = cfg.get("DeviceInfo", "VendorName", fallback="").lower()
    is_maxon = "maxon" in vendor
    desc_count = {}

    for sec_name in cfg.sections():
        m = SECTION_RE.match(sec_name)
        if not m:
            continue

        idx_hex = m.group("idx").upper()
        sub = m.group("sub")
        idx = int(idx_hex, 16)

        if index_has_subs.get(idx_hex, False) and sub is None:
            continue

        sec = cfg[sec_name]

        if sub is None:
            subidx = 0
            sub_str = "0x0"
        else:
            subidx = int(sub)
            sub_str = f"0x{subidx:x}"

        desc = sec.get("ParameterName", "").strip()
        datatype = decode_datatype(sec.get("DataType", "").strip())
        access_raw = sec.get("AccessType", "").strip()
        access = decode_access(access_raw)

        pdomap = sec.get("PDOMapping", "").strip()
        objflags_raw = sec.get("ObjFlags", "").strip()

        suf = pdo_suffix(idx)
        if suf and desc and "pdo" not in desc.lower():
            desc = f"{desc}{suf}"

        rxpdo = 0
        txpdo = 0

        if in_ranges(idx, COMM_RANGES):
            rxpdo = 0
            txpdo = 0
        else:
            if pdomap == "1":
                ar = access_raw.lower()
                if "w" in ar:
                    rxpdo = 1
                if "r" in ar:
                    txpdo = 1
            elif objflags_raw:
                try:
                    flags = int(objflags_raw, 0)
                except ValueError:
                    flags = 0
                if flags & 0x1:
                    rxpdo = 1
                if flags & 0x2:
                    txpdo = 1

            if is_maxon and idx in MAXON_KNOWN_INFO:
                rxpdo = 0
                txpdo = 0

        base_desc = desc or ""
        cnt = desc_count.get(base_desc, 0)
        if cnt >= 1 and base_desc:
            desc = f"{base_desc} ({cnt + 1})"
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

    rows.sort(key=lambda r: (int(r["index"], 16), int(r["subindex"], 16)))
    return rows


# ---------- generación del header ----------

def generate_c_identifier(description: str) -> str:
    description = description.replace('(', '').replace(')', '')
    identifier = re.sub(r'[^a-zA-Z0-9]', '_', description)
    if not identifier:
        identifier = "obj"
    if identifier[0].isdigit():
        identifier = f"_{identifier}"
    return identifier.lower()


def generate_ctype(t: str) -> str:
    unknown = "unknown_t"
    return {
        "ARRAY": unknown,
        "BOOLEAN": "bool",
        "INTEGER8": "int8_t",
        "INTEGER16": "int16_t",
        "INTEGER32": "int32_t",
        "INTEGER64": "int64_t",
        "REAL32": "float",
        "REAL64": "double",
        "RECORD": unknown,
        "STRUCT": unknown,
        "DOMAIN": unknown,
        "OCTET_STRING": unknown,
        "UNICODE_STRING": unknown,
        "UNSIGNED8": "uint8_t",
        "UNSIGNED16": "uint16_t",
        "UNSIGNED32": "uint32_t",
        "UNSIGNED64": "uint64_t",
        "STRING4": "string4_t",
        "STRING8": "string8_t",
    }.get(t, unknown)


def enrich_rows(rows):
    enriched = []
    for row in rows:
        desc = row["description"]
        c_identifier = generate_c_identifier(desc)
        access = row["access"]
        getter = f"get_{c_identifier.lstrip('_')}" if access in {"RW", "RO", "CONST"} else "NA"
        setter = f"set_{c_identifier.lstrip('_')}" if access in {"RW", "WO"} else "NA"
        enriched.append({
            **row,
            "c_identifier": c_identifier,
            "getter": getter,
            "setter": setter,
            "ctype": generate_ctype(row["type"]),
        })
    return enriched


def write_csv(rows, out_path: Path):
    fieldnames = ["index", "subindex", "description", "type", "access", "rxPDO", "txPDO"]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, delimiter=';')
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in fieldnames})


def write_header(rows, out_path: Path, macro_name: str, source_name: str):
    lines = []
    for r in rows:
        desc = r["description"]
        if desc.startswith("Internal "):
            continue
        lines.append(
            f'    OBJ({r["index"]}, {r["subindex"]}, "{desc}", {r["c_identifier"]}, {r["ctype"]}, {r["rxPDO"]}, {r["txPDO"]}, {r["getter"]}, {r["setter"]}) \\\n'
        )

    if not lines:
        raise ValueError("No hay objetos para escribir en la cabecera")

    lines[-1] = lines[-1].rstrip(" \\\n") + "\n"

    with out_path.open("w", encoding="utf-8", newline="") as f:
        f.write(
            "#pragma once\n"
            f"// DO NOT EDIT. This file is autogenerated from '{source_name}'\n"
            "#include <stdint.h>\n"
            "#include <stdbool.h>\n"
            "// Format: OBJ(index, subindex, desc, identifier, type, rxPDO, txPDO, getter, setter)\n"
            f"#define {macro_name}(OBJ) \\\n"
        )
        f.writelines(lines)


def main():
    ap = argparse.ArgumentParser(
        description="Genera directamente la cabecera OD a partir de una EDS y, opcionalmente, también el CSV intermedio"
    )
    ap.add_argument("eds", help="archivo .eds de entrada")
    ap.add_argument("-o", "--output", default=None, help="archivo .h de salida (por defecto <eds>_od.h)")
    ap.add_argument("--csv", dest="csv_output", default=None,
                    help="ruta del CSV a generar opcionalmente; si se usa sin valor no aplica, indicar una ruta")
    ap.add_argument("--macro", default=None,
                    help="nombre del macro del diccionario (por defecto <stem>_OD en mayúsculas)")
    args = ap.parse_args()

    eds_path = Path(args.eds)
    if not eds_path.exists():
        raise FileNotFoundError(f"Archivo EDS no encontrado: {eds_path}")

    cfg = parse_eds(eds_path)
    rows = eds_to_rows(cfg)
    rows = enrich_rows(rows)

    out_path = Path(args.output) if args.output else Path("src") / f"{eds_path.stem}_od.h"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    macro_name = args.macro or f"{eds_path.stem.upper()}_OD"
    write_header(rows, out_path, macro_name, eds_path.name)
    print(f"[+] Generada cabecera: {out_path}")

    if args.csv_output:
        csv_path = Path(args.csv_output)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        write_csv(rows, csv_path)
        print(f"[+] Generado CSV: {csv_path}")

    print(f"[+] Objetos exportados: {len(rows)}")


if __name__ == "__main__":
    main()
