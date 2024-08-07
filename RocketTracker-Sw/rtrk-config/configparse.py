import pyjson5
from abc import ABC, abstractmethod
from os.path import join

DEFAULT_PARAMETER_INFO = {
    "!name",
    "!type",
    "!default",
}

DEFAULT_COLLECTION_INFO = {
    '!defs',
    "!name"
}

root_dir = ""


class TypeBase(ABC):
    @abstractmethod
    def parse_value(self, nativeval: any):
        pass

    @abstractmethod
    def parse_type(self, nativeval: any):
        pass


class BoolType(TypeBase):
    def parse_type(self, nativeval: any):
        assert nativeval == 'bool'
        return self

    def __repr__(self) -> str:
        return "BoolType"

    def parse_value(self, nativeval: any):
        if not isinstance(nativeval, bool):
            return None
        return nativeval


class IntType(TypeBase):
    def __init__(self) -> None:
        self.min = None
        self.max = None
        self.validvals = None

    def parse_type(self, nativeval):
        if isinstance(nativeval, str):
            if ('{' in nativeval):
                self.validvals = set([int(v) for v in nativeval.split(
                    '{')[1].split('}')[0].split(',')])
            else:
                base = nativeval.split('(')
                params = [None, None]
                if (len(base) > 1):
                    params = [
                        (int(i) if i.isnumeric() else None)
                        for i in base[1][:-1].split(',')
                    ]
                    assert len(params) == 2

                self.min = params[0]
                self.max = params[1]
        else:
            raise Exception("InvalidTypedef")
        return self

    def __repr__(self) -> str:
        inner = ""
        if self.validvals is None:
            inner = f"min={self.min}, max={self.max}"
        else:
            inner = f"valid={self.validvals}"
        return f"int({inner})"

    def parse_value(self, nativeval: any):
        if not isinstance(nativeval, int):
            return None
        valid = True
        if (self.min is not None):
            valid &= nativeval >= self.min
        if (self.max is not None):
            valid &= nativeval <= self.max
        if (self.validvals is not None):
            valid &= nativeval in self.validvals
        if not valid:
            return None
        return nativeval


class FloatType(TypeBase):
    def __init__(self) -> None:
        self.min = None
        self.max = None
        self.validvals = None

    def parse_type(self, nativeval):
        if isinstance(nativeval, str):
            if ('{' in nativeval):
                self.validvals = set([float(v) for v in nativeval.split(
                    '{')[1].split('}')[0].split(',')])
            else:
                base = nativeval.split('(')
                params = [None, None]
                if (len(base) > 1):
                    params = [
                        (float(i) if (not i.isalpha()) else None)
                        for i in base[1][:-1].split(',')
                    ]
                    assert len(params) == 2

                self.min = params[0]
                self.max = params[1]
        else:
            raise Exception("InvalidTypedef")
        return self

    def __repr__(self) -> str:
        inner = ""
        if self.validvals is None:
            inner = f"min={self.min}, max={self.max}"
        else:
            inner = f"valid={self.validvals}"
        return f"float({inner})"

    def parse_value(self, nativeval: any):
        if not isinstance(nativeval, float):
            return None
        valid = True
        if (self.min is not None):
            valid &= nativeval >= self.min
        if (self.max is not None):
            valid &= nativeval <= self.max
        if (self.validvals is not None):
            valid &= nativeval in self.validvals
        if (not valid):
            return None
        return nativeval


class StringType(TypeBase):
    def parse_type(self, nativeval: any):
        if isinstance(nativeval, str):
            assert nativeval.startswith('str')
            # self.maxlen = int(nativeval.split('(')[1].split(')')[0])
            self.maxlen = 64
        else:
            raise Exception("InvalidTypedef")
        return self

    def __repr__(self) -> str:
        return f"string(maxlen={self.maxlen})"

    def parse_value(self, nativeval: any):
        if not (isinstance(nativeval, str) and len(nativeval) <= self.maxlen):
            return None
        return nativeval


class EnumType(TypeBase):
    def __init__(self) -> None:
        self.values: list[str] = []

    def parse_type(self, nativeval: any):
        assert isinstance(nativeval, list)
        self.values = nativeval
        return self

    def __repr__(self) -> str:
        return f"enum({', '.join(self.values)})"

    def parse_value(self, nativeval: any):
        if not (isinstance(nativeval, str) and nativeval in self.values):
            return None
        return nativeval


def get_type(t: any, customtypes={}):
    if isinstance(t, list):
        return EnumType().parse_type(t)
    else:
        assert isinstance(t, str)
        base = t.split('(')
        if (base[0] == 'int'):
            return IntType().parse_type(t)
        elif (base[0] == 'float'):
            return FloatType().parse_type(t)
        elif (base[0] == 'string'):
            return StringType().parse_type(t)
        elif base[0] == 'bool':
            return BoolType().parse_type(t)
        elif (t in customtypes):
            return customtypes[t]


class ParameterBase:
    def __init__(self, d: dict, path, customtypes={}) -> None:
        self.path = path
        if ("!name" in d):
            self.name = d['!name']
        else:
            # raise Exception("NoNameError")
            self.name = None

        if ("!type" in d):
            self.type = get_type(d['!type'], customtypes)
        else:
            raise Exception("NoTypeError")

        if ("!default" in d):
            self.default = self.type.parse_value(d['!default'])
        else:
            raise Exception("NoDefaultError")

        # self.displayname = d.get("!display", self.name)
        # self.description = d.get('!description', None)

        self.data = {
            k: v for k, v in d.items()
            if k not in DEFAULT_PARAMETER_INFO
        }

    def __repr__(self) -> str:
        return f"{self.path} type: {self.type} default: {self.default}"


class CollectionBase:
    def __init__(self) -> None:
        self.path = ''
        self.types = {}
        self.items = {}
        self.data = {}
        pass


def parsecollection(k, v, path=None, customtypes={}):
    global root_dir

    c_types = customtypes.copy()
    newpath = k if path is None else path+f".{k}"

    coll = CollectionBase()

    if '!defs' in v:
        for typename, typedef in v['!defs'].items():
            coll.types[typename] = get_type(typedef, customtypes)

    c_types |= coll.types
    assert '!name' in v and isinstance(v['!name'], str)
    coll.path = newpath

    coll.data = {
        kk: vv for kk, vv in v.items()
        if k not in DEFAULT_COLLECTION_INFO
    }

    for key, val in v.items():
        if (key.startswith('!')):
            continue

        if isinstance(val, str):
            # Custom command
            assert val.startswith('!include')
            if (val.startswith('!include')):
                fname = val.split('(')[1].split(')')[0]
                val = pyjson5.load(open(join(root_dir, fname)))

        coll.items[key] = parseconfig(
            key, val, newpath, c_types)

    return coll


# TODO: Output of custom types as a lobal dict?
def parseconfig(k, v, path='config', customtypes={}):
    c_types = customtypes.copy()
    assert isinstance(v, dict)

    if '!type' in v:
        # parameter
        return ParameterBase(v, path+f".{k}", customtypes=c_types)
    elif '!name' in v:
        # Collection
        return parsecollection(k, v, customtypes=c_types, path=path)
    else:
        print("Error")

    return None


def print_tree(c, level=0):
    if isinstance(c, CollectionBase):
        print('\t'*level + f"{c.path}:")
        for k, v in c.items.items():
            print_tree(v, level)
    elif isinstance(c, ParameterBase):
        print('\t'*level + c.path, c.type)
    else:
        pass


def flatten_config_types(c):
    ts = {}
    if isinstance(c, CollectionBase):
        for k, v in c.types.items():
            ts[k] = v

        for k, v in c.items.items():
            ts |= flatten_config_types(v)

    return ts


def flatten_config_values(c) -> list[ParameterBase]:
    vs = []
    if isinstance(c, CollectionBase):
        for k, v in c.items.items():
            vs += flatten_config_values(v)
    elif isinstance(c, ParameterBase):
        vs.append(c)
    else:
        print("Flattener error")

    return vs


def parse_configuration(file, root=""):
    global root_dir
    root_dir = root
    JSON = pyjson5.load(file)
    return parsecollection('config', JSON)


if __name__ == "__main__":
    c = parse_configuration(open("./tracker_config.jsonc"))
    # print_tree(c)
    print("Values:")
    for v in flatten_config_values(c):
        print(f"\t{v}")
    print("Types:")
    print(flatten_config_types(c))
