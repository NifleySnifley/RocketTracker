from configparse import *
from csnake import CodeWriter, Struct, Enum, Function, Variable, Typecast
import base64
from hashlib import shake_128
c = parse_configuration(
    open("./configfiles/tracker_config.jsonc"), root="./configfiles/")

types = flatten_config_types(c)
values = flatten_config_values(c)

cw = CodeWriter()

TYPES = [
    "IntType",
    "FloatType",
    "StringType",
    "EnumType",
    "BoolType"
]


def encode_key(k: str):
    kenc = base64.b64encode(shake_128(k.encode('ascii')).digest(9))
    # print(len(kenc))
    return kenc.decode('ascii')


configtypes = Enum("config_type_t", typedef=True)
for t in TYPES:
    configtypes.add_value(t)

cw.add_enum(configtypes)

for tname, t in types.items():
    if isinstance(t, EnumType):
        typename = f"{tname}_t"
        e = Enum(typename, typedef=True)
        for v in t.values:
            v_clean = ''.join(
                [(c.upper() if (c.lower() in "1234567890abcdefghijklmnopqrstuvwxyz") else '_') for c in v.upper()])
            if (v_clean[0].isnumeric()):
                v_clean = '_'+v_clean
            e.add_value(v_clean)

        print(e.generate_declaration())
        cw.add_enum(e)
        # print(basename)

defs = ['\n']
enkeys = set()
for v in values:
    vpath_clean = v.path.replace(".", "_")
    print(vpath_clean)
    defval = v.default
    if (isinstance(v.type, EnumType)):
        defval = v.type.values.index(defval)

        # Test for duplicate keys after hashing
    enkey = encode_key(v.path)
    assert enkey not in enkeys
    enkeys.add(enkey)

    defs.append(f"#define {vpath_clean.upper()}_KEY \"{enkey}\"")
    if (isinstance(v.type, BoolType)):
        defs.append(
            f"#define {vpath_clean.upper()}_DEFAULT {'true' if defval else 'false'}")
    else:
        defs.append(f"#define {vpath_clean.upper()}_DEFAULT {defval}")
    defs.append('')

cfg_entry = Struct("config_entry_t", typedef=True)
cfg_entry.add_variable(Variable("type", "config_type_t"))
cfg_entry.add_variable(Variable("name", "const char*"))
cfg_entry.add_variable(Variable("encoded_key", "const char*"))
cw.add_struct(cfg_entry)

# Manifest
var = Variable(
    "CONFIG_MANIFEST",
    primitive="config_entry_t",
    value=[
        {
            "type": Typecast(TYPES.index(v.type.__class__.__name__), "config_type_t"),
            "name": v.path,
            "encoded_key": encode_key(v.path)
        }
        for v in values
    ],
    qualifiers="const"
)
cw.add_variable_initialization(var)

cw.add('\n'.join(defs))
cw.write_to_file("./generated/config.h")
