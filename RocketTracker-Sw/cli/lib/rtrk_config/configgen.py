from hashlib import shake_128
import base64


def encode_key(k: str):
    kenc = base64.b64encode(shake_128(k.encode('ascii')).digest(9))
    # print(len(kenc))
    return kenc.decode('ascii')


if __name__ == "__main__":
    from configparse import *
    from csnake import CodeWriter, Struct, Enum, Function, Variable, Typecast, FormattedLiteral
    from csnake.cconstructs import CFloatLiteral

    c = parse_configuration(
        open("./configfiles/tracker_config.jsonc"), root="./configfiles/")

    types = flatten_config_types(c)
    values = flatten_config_values(c)

    Nvalues = len(values)

    cw = CodeWriter()

    cw.add_line("#ifndef CONFIG_GENERATED_H")
    cw.add_line("#define CONFIG_GENERATED_H")

    cw.include("stdint.h")

    TYPES = [
        "IntType",
        "FloatType",
        "StringType",
        "EnumType",
        "BoolType"
    ]

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

    cw.add_lines("""typedef union config_value_t {
    float float_value;
    int32_t int_value;
    char* string_value;
    bool bool_value;
    int32_t enum_value;
    } config_value_t;""")

    cfg_entry = Struct("config_entry_t", typedef=True)
    cfg_entry.add_variable(Variable("type", "config_type_t"))
    cfg_entry.add_variable(Variable("key", "const char*"))
    cfg_entry.add_variable(Variable("hashed_key", "const char*"))
    cfg_entry.add_variable(Variable("default_value", "config_value_t"))
    cw.add_struct(cfg_entry)

    cw.add('\n'.join(defs))
    cw.add_line(f"extern const config_entry_t CONFIG_MANIFEST[{Nvalues}];")

    cw.add_line("#endif")

    cw.add_line("#ifdef CONFIG_GENERATED_IMPL")

    def convert_value(type, value):
        v = None
        if (isinstance(type, EnumType)):
            v = Typecast(type.values.index(value), "int32_t")
        elif isinstance(type, FloatType):
            v = Typecast(value, "float")
        elif isinstance(type, IntType):
            v = Typecast(value, "int32_t")
        elif isinstance(type, BoolType):
            v = Typecast(value, "bool")
        elif isinstance(type, StringType):
            v = Typecast(value, "const char*")
        else:
            v = value
        return Typecast(v, "config_value_t")

    # Manifest
    var = Variable(
        "CONFIG_MANIFEST",
        primitive="config_entry_t",
        value=[
            {
                "type": Typecast(TYPES.index(v.type.__class__.__name__), "config_type_t"),
                "key": v.path,
                "hashed_key": encode_key(v.path),
                "default_value": convert_value(v.type, v.default)
            }
            for v in values
        ],
        qualifiers="const"
    )
    cw.add_variable_initialization(var)

    cw.add_line("#endif")

    cw.write_to_file("./generated/config_generated.h")
