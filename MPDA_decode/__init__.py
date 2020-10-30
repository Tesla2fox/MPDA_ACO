


# from  MPDA_decode import  decode


from MPDA_decode import  basicDecdoe
from MPDA_decode.instance import  Instance
from MPDA_decode.MPDA_de_decode_continue import  MPDA_DE_decode

# __all__ = ["algorithms", "benchmarks", "util", "task", "Runner"]

__all__ =["basicDecdoe","Instance","MPDA_DE_decode"]
__project__ = "MPDA_decode"
__version__ = "0.0.01"

VERSION = "{0} v{1}".format(__project__, __version__)


if __name__ == '__main__':
    print(basicDecdoe)
    print(Instance)